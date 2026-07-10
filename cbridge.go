package main

/*
#cgo CFLAGS: -I${SRCDIR}/third_party/Livox-SDK2/include
#cgo CXXFLAGS: -I${SRCDIR}/third_party/Livox-SDK2/include -std=c++11
#cgo LDFLAGS: -L${SRCDIR}/third_party/Livox-SDK2/build/sdk_core -llivox_lidar_sdk_static -lstdc++ -lpthread
#include <stdlib.h>
#include <stdbool.h>
#include "cbridge.h"
#include "livox_lidar_def.h"
*/
import "C"
import (
	"encoding/binary"
	"fmt"
	"sync"
	"sync/atomic"
	"unsafe"
)

// Point data extracted from SDK callbacks.
type pointData struct {
	X, Y, Z      int32
	Reflectivity uint8
	Tag          uint8
}

// IMU reading extracted from SDK callbacks.
type imuReading struct {
	GyroX, GyroY, GyroZ float32
	AccX, AccY, AccZ     float32
	Timestamp            uint64
}

// frameBuffer double-buffers point cloud frames.
type frameBuffer struct {
	mu             sync.Mutex
	writing        []pointData
	reading        []pointData
	frameCnt       uint8
	writeTimestamp uint64
	readTimestamp  uint64
	ready          chan struct{}
	initialized    bool
}

var (
	frames    = &frameBuffer{ready: make(chan struct{}, 1)}
	latestIMU atomic.Value // stores *imuReading
)

//export goPointCloudCallback
func goPointCloudCallback(handle C.uint32_t, devType C.uint8_t, data *C.LivoxLidarEthernetPacket) {
	pkt := unsafe.Pointer(data)
	dotNum := uint16(C.pkt_dot_num(pkt))
	dataType := uint8(C.pkt_data_type(pkt))

	var tsBuf [8]byte
	C.pkt_timestamp(pkt, (*C.uint8_t)(unsafe.Pointer(&tsBuf[0])))
	ts := binary.LittleEndian.Uint64(tsBuf[:])

	// Only handle high-res cartesian for now
	if dataType != C.kLivoxLidarCartesianCoordinateHighData {
		return
	}

	frames.mu.Lock()
	defer frames.mu.Unlock()

	// Time-based framing: accumulate 100ms of data per frame (~10Hz)
	const frameIntervalNs = 100_000_000 // 100ms
	if frames.initialized && ts-frames.writeTimestamp >= frameIntervalNs {
		frames.reading, frames.writing = frames.writing, frames.reading[:0]
		frames.readTimestamp = frames.writeTimestamp
		select {
		case frames.ready <- struct{}{}:
		default:
		}
	}

	frames.initialized = true
	if len(frames.writing) == 0 {
		frames.writeTimestamp = ts
	}

	// Copy points from packet
	base := C.pkt_data(pkt)
	pointSize := C.sizeof_LivoxLidarCartesianHighRawPoint
	for i := uint16(0); i < dotNum; i++ {
		p := (*C.LivoxLidarCartesianHighRawPoint)(unsafe.Add(base, uintptr(i)*uintptr(pointSize)))
		frames.writing = append(frames.writing, pointData{
			X:            int32(p.x),
			Y:            int32(p.y),
			Z:            int32(p.z),
			Reflectivity: uint8(p.reflectivity),
			Tag:          uint8(p.tag),
		})
	}
}

//export goImuCallback
func goImuCallback(handle C.uint32_t, devType C.uint8_t, data *C.LivoxLidarEthernetPacket) {
	pkt := unsafe.Pointer(data)
	dataType := uint8(C.pkt_data_type(pkt))
	if dataType != C.kLivoxLidarImuData {
		return
	}

	var tsBuf [8]byte
	C.pkt_timestamp(pkt, (*C.uint8_t)(unsafe.Pointer(&tsBuf[0])))
	ts := binary.LittleEndian.Uint64(tsBuf[:])

	p := (*C.LivoxLidarImuRawPoint)(C.pkt_data(pkt))
	gx, gy, gz := float32(p.gyro_x), float32(p.gyro_y), float32(p.gyro_z)
	ax, ay, az := float32(p.acc_x), float32(p.acc_y), float32(p.acc_z)

	latestIMU.Store(&imuReading{
		GyroX: gx, GyroY: gy, GyroZ: gz,
		AccX: ax, AccY: ay, AccZ: az,
		Timestamp: ts,
	})

	// Feed dead reckoner at 200Hz
	dr.update(gx, gy, gz, ax, ay, az, ts)
}

//export goInfoChangeCallback
func goInfoChangeCallback(handle C.uint32_t, info *C.LivoxLidarInfo) {
	sn := C.GoStringN(&info.sn[0], 16)
	ip := C.GoStringN(&info.lidar_ip[0], 16)

	fmt.Printf("Livox device connected: handle=%d sn=%s ip=%s\n", uint32(handle), sn, ip)

	sdkMgr.addHandle(uint32(handle))

	// Set to normal work mode and enable IMU
	C.bridge_set_work_mode(handle, C.int(C.kLivoxLidarNormal))
	C.bridge_enable_imu(handle)
}

// Go wrappers for calling C bridge functions.

func bridgeInit(cfgPath, hostIP string) error {
	cPath := C.CString(cfgPath)
	defer C.free(unsafe.Pointer(cPath))
	cHost := C.CString(hostIP)
	defer C.free(unsafe.Pointer(cHost))

	if C.bridge_init(cPath, cHost) != 0 {
		return fmt.Errorf("LivoxLidarSdkInit failed")
	}
	return nil
}

func bridgeStart() error {
	if C.bridge_start() != 0 {
		return fmt.Errorf("LivoxLidarSdkStart failed")
	}
	return nil
}

func bridgeRegisterCallbacks() {
	C.bridge_register_callbacks()
}

func bridgeUninit() {
	C.bridge_uninit()
}
