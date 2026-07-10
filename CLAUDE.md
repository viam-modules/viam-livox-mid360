# Livox Mid-360 Viam Module

Go module implementing Viam Camera (point cloud) and MovementSensor (IMU) for the Livox Mid-360 LiDAR, using cgo to wrap Livox-SDK2.

## Project Structure

```
livox-mid-360/
├── CLAUDE.md
├── go.mod / go.sum
├── main.go                  # module entry, registers both models, calls ModularMain
├── config.go                # shared Config struct + Validate()
├── sdk_manager.go           # SDK2 lifecycle singleton (init/start/uninit), ref-counted
├── cbridge.go               # cgo declarations + //export callbacks
├── cbridge.c                # C shim: SDK2 callback registrations, calls exported Go funcs
├── cbridge.h                # C shim header
├── camera.go                # Camera component: NextPointCloud(), Properties()
├── movement_sensor.go       # MovementSensor component: AngularVelocity(), LinearAcceleration()
├── Makefile                 # builds Livox-SDK2 + Go module
└── mid360_config.json       # template SDK2 JSON config (generated at runtime from Viam config)
```

## Components

### 1. Camera — `viam-labs:livox:mid360`

Implements the Viam Camera interface. Only `NextPointCloud()` and `Properties()` are meaningful; `Images()` returns unimplemented.

```go
type Camera interface {
    resource.Resource
    resource.Shaped
    Images(ctx, filterSourceNames, extra) ([]NamedImage, ResponseMetadata, error)  // unimplemented
    NextPointCloud(ctx, extra) (pointcloud.PointCloud, error)                      // main entry
    Properties(ctx) (Properties, error)                                            // SupportsPCD: true
}
```

**NextPointCloud** returns one complete frame's worth of accumulated points. The Mid-360 sends ~200k pts/sec across many UDP packets; a "frame" at 10Hz is ~20k points.

### 2. MovementSensor — `viam-labs:livox:mid360-imu`

Implements the Viam MovementSensor interface for the built-in IMU.

```go
type MovementSensor interface {
    resource.Sensor
    resource.Resource
    Position(ctx, extra) (*geo.Point, float64, error)                  // unimplemented
    LinearVelocity(ctx, extra) (r3.Vector, error)                      // unimplemented
    AngularVelocity(ctx, extra) (spatialmath.AngularVelocity, error)   // gyro_x/y/z
    LinearAcceleration(ctx, extra) (r3.Vector, error)                  // acc_x/y/z
    CompassHeading(ctx, extra) (float64, error)                        // unimplemented
    Orientation(ctx, extra) (spatialmath.Orientation, error)            // unimplemented
    Properties(ctx, extra) (*Properties, error)                        // reports supported methods
    Accuracy(ctx, extra) (*Accuracy, error)                            // unimplemented
}
```

## SDK Manager (Singleton)

Livox-SDK2 has global init/uninit. Both components share a single SDK instance.

```
sdkManager.Acquire(config)  // first call: init + start SDK; subsequent: ref++
sdkManager.Release()        // ref--; if 0: uninit SDK
```

The manager:
- Generates a temporary JSON config from the Viam component config
- Calls LivoxLidarSdkInit(path) + LivoxLidarSdkStart()
- Registers C callbacks once via cbridge.c
- Tracks connected device handles via InfoChangeCallback

## cgo Bridge Design

C callbacks run on SDK threads, not Go goroutines. The bridge uses `//export` Go functions that the C shim calls.

### Callback flow:

```
SDK2 thread                          Go side
───────────                          ───────
PointCloudCallback (C)
  → go_point_cloud_callback (//export Go func)
      → copies packet data into Go-managed frame buffer
      → when frame_cnt changes: swap double buffer, signal channel

ImuDataCallback (C)
  → go_imu_callback (//export Go func)
      → atomically stores latest IMU reading

InfoChangeCallback (C)
  → go_info_change_callback (//export Go func)
      → stores device handle
      → calls SetLivoxLidarWorkMode(handle, kLivoxLidarNormal)
```

### Double buffer for point cloud frames:

```go
type frameBuffer struct {
    mu            sync.Mutex
    writing       []pointData    // currently accumulating from callbacks
    reading       []pointData    // complete frame, served by NextPointCloud()
    frameCnt      uint8          // current frame_cnt from SDK packets
    writeTimestamp uint64        // timestamp from first packet in current writing frame
    readTimestamp  uint64        // timestamp of the completed reading frame (ns)
    ready         chan struct{}  // signaled when a new complete frame is swapped in
}
```

The Mid-360 does not increment `frame_cnt` in packet headers (always 0). Instead, we use **time-based framing**: when the timestamp gap between the first packet in the current accumulation buffer and a new packet exceeds 100ms, the buffer is complete:
- Lock, swap writing↔reading, signal `ready`

`NextPointCloud()` waits on `ready` (with ctx deadline), then converts reading buffer to `pointcloud.PointCloud`.

### Point conversion:

```go
// SDK2 gives mm as int32; Viam expects meters as float64
x := float64(raw.x) / 1000.0
y := float64(raw.y) / 1000.0
z := float64(raw.z) / 1000.0
pc.Set(r3.Vector{X: x, Y: y, Z: z}, pointcloud.NewValueData(int(raw.reflectivity)))
```

## Livox SDK2 Key API

```c
// Lifecycle
bool LivoxLidarSdkInit(const char* path, const char* host_ip, const LivoxLidarLoggerCfgInfo* log_cfg_info);
bool LivoxLidarSdkStart();
void LivoxLidarSdkUninit();

// Callbacks
void SetLivoxLidarPointCloudCallBack(LivoxLidarPointCloudCallBack cb, void* client_data);
void SetLivoxLidarImuDataCallback(LivoxLidarImuDataCallback cb, void* client_data);
void SetLivoxLidarInfoChangeCallback(LivoxLidarInfoChangeCallback cb, void* client_data);

// Control (called from InfoChangeCallback once device is discovered)
livox_status SetLivoxLidarWorkMode(uint32_t handle, LivoxLidarWorkMode work_mode, ...);
```

### Data structures:

```c
// Point: high-res cartesian (14 bytes/point)
typedef struct {
  int32_t x, y, z;       // millimeters
  uint8_t reflectivity;
  uint8_t tag;
} LivoxLidarCartesianHighRawPoint;

// IMU (24 bytes)
typedef struct {
  float gyro_x, gyro_y, gyro_z;  // rad/s
  float acc_x, acc_y, acc_z;     // g
} LivoxLidarImuRawPoint;

// Packet header
typedef struct {
  uint8_t version;
  uint16_t length, time_interval, dot_num, udp_cnt;
  uint8_t frame_cnt, data_type, time_type, rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  uint8_t data[1];  // flexible array of points
} LivoxLidarEthernetPacket;

// Device type: kLivoxLidarTypeMid360 = 9
// Work modes: kLivoxLidarNormal = 0x01, kLivoxLidarSleep = 0x03
```

## Time Synchronization

The Mid-360 packet header carries timing info:
```c
// In LivoxLidarEthernetPacket:
uint8_t  time_type;     // sync source (0=no sync, 1=PTP, ...)
uint8_t  timestamp[8];  // nanosecond timestamp (uint64 LE)
uint16_t time_interval; // interval between points in 0.1μs units
```

Device state also reports:
```c
uint64_t local_time_now;   // device clock
uint64_t last_sync_time;   // last PTP/GPS sync event
int64_t  time_offset;      // offset from sync source
uint8_t  time_sync_type;   // active sync method
```

### What we track

1. **Per-frame timestamp** — store the `timestamp[8]` from the first packet in each accumulated frame. This becomes the frame's capture time.
2. **Per-IMU timestamp** — store alongside each IMU reading so AngularVelocity/LinearAcceleration reflect when the measurement was taken.
3. **Sync status** — read `time_sync_type` from device state on connect to log whether PTP is active.

### Where timestamps surface in Viam

- **Camera**: `Images()` returns `resource.ResponseMetadata{CapturedAt: time.Time}`. Even though we primarily use `NextPointCloud()`, we populate `CapturedAt` from the frame timestamp so any caller gets accurate timing.
- **MovementSensor**: `Readings()` (from `resource.Sensor`) can include a `"timestamp"` key with the packet time.
- **Both**: If PTP is active, timestamps are wall-clock. If not, they're device-uptime relative — we note this in logs on connect.

### Sync modes (config)

- `"time_sync": "none"` (default) — use device internal clock, timestamps are relative.
- `"time_sync": "ptp"` — assume PTP master is on the network. The Mid-360 handles PTP at the hardware level; we just verify `time_sync_type` indicates PTP lock and warn if not.
- `"time_sync": "gps"` — periodically inject RMC strings via `SetLivoxLidarRmcSyncTime()`. Requires a GPS source (future, not in initial build).

### Frame-IMU correlation

Both point cloud and IMU callbacks store timestamps from the same clock source. Downstream consumers can correlate them by comparing timestamps. We don't do interpolation or fusion in the module — that's the job of higher-level Viam services.

## Viam Module Config

Passed via Viam app JSON. Both components share the same config:

```json
{
  "components": [
    {
      "name": "lidar",
      "api": "rdk:component:camera",
      "model": "viam-labs:livox:mid360",
      "attributes": {
        "sensor_ip": "192.168.1.1xx",
        "host_ip": "",
        "time_sync": "ptp"
      }
    },
    {
      "name": "imu",
      "api": "rdk:component:movement_sensor",
      "model": "viam-labs:livox:mid360-imu",
      "attributes": {
        "sensor_ip": "192.168.1.1xx",
        "host_ip": "",
        "time_sync": "ptp"
      }
    }
  ]
}
```

The module generates the SDK2 JSON config at init time from these attributes.

## Build

### Prerequisites
- Livox-SDK2 built and installed (`cmake .. && make && sudo make install`)
- Go 1.21+

### cgo linking
```go
// #cgo CFLAGS: -I/usr/local/include
// #cgo LDFLAGS: -L/usr/local/lib -llivox_lidar_sdk_static -lstdc++ -lpthread
// #include "cbridge.h"
import "C"
```

### Makefile targets
- `make sdk`: clone + build Livox-SDK2
- `make build`: build the Go module binary
- `make all`: sdk + build

## Test Plan

### Prerequisites
- Mid-360 powered on and connected via Ethernet
- Host machine on same subnet (default Mid-360 IP: 192.168.1.1xx)
- Confirm connectivity: `ping <sensor_ip>`

### Phase 1: Standalone binary smoke test (no Viam server)

Write a minimal test harness (`cmd/test/main.go`) that:
1. Initializes the SDK manager directly with a hardcoded config
2. Waits for the InfoChangeCallback to fire (device discovery)
3. Calls `NextPointCloud` equivalent (reads from the frame buffer)
4. Prints IMU readings from the atomic store

**Pass criteria:**
- [ ] "Livox device connected" message prints within ~5s
- [ ] Point cloud frames arrive (~10Hz)
- [ ] Frame sizes are reasonable (~15k-25k points per frame at 10Hz)
- [ ] Point coordinates are in plausible range (not all zeros, not NaN, within ~200m)
- [ ] IMU data arrives and updates
- [ ] Gyro at rest: all axes near zero (< 0.1 rad/s)
- [ ] Accel at rest: one axis ~1g (9.8 m/s²), others near zero
- [ ] Timestamps are non-zero and monotonically increasing
- [ ] Clean shutdown (no crash, no hang)

### Phase 2: Viam module integration

Run as a Viam module with a local `viam-server`:
```bash
viam-server -config test_config.json
```

With a config that registers both components. Then use the Viam CLI or app to:

1. **Camera component**
   - [ ] Component appears in the control tab
   - [ ] `GetPointCloud` returns PCD data
   - [ ] Point cloud visualizes correctly in the Viam app (3D view)
   - [ ] Repeated calls return fresh frames (not stale data)

2. **MovementSensor component**
   - [ ] Component appears in the control tab
   - [ ] `GetReadings` returns angular_velocity and linear_acceleration
   - [ ] Values update in real time
   - [ ] Properties reports AngularVelocitySupported=true, LinearAccelerationSupported=true
   - [ ] Unimplemented methods (Position, CompassHeading, etc.) return proper errors

3. **Lifecycle**
   - [ ] Module starts cleanly when viam-server launches
   - [ ] Reconfigure (change sensor_ip) works or fails gracefully
   - [ ] Module shuts down cleanly when viam-server stops (no zombie processes, no UDP socket leaks)
   - [ ] Removing one component doesn't break the other (ref counting)

### Phase 3: Edge cases

- [ ] Module starts before lidar is powered on — should wait, connect when lidar comes up
- [ ] Lidar power-cycled while module is running — should recover or error cleanly
- [ ] Multiple rapid `NextPointCloud` calls — should not deadlock or return partial frames
- [ ] Context cancellation mid-wait — should return promptly, not block

### What we're NOT testing yet
- PTP time sync verification (need PTP master on network)
- GPS/RMC time sync (not implemented)
- Multi-lidar (single device for now)
- Performance benchmarking / sustained throughput
- Cross-compilation / Linux arm64 build
