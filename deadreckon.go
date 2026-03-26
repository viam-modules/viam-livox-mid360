package main

import (
	"math"
	"sync"
)

// deadReckoner integrates IMU data at 200Hz to estimate pose.
// Orientation is tracked via gyro integration (rotation matrix).
// Position is double-integrated from gravity-compensated acceleration.
// This drifts quickly but provides a reasonable prior between scans.
type deadReckoner struct {
	mu sync.Mutex

	// Orientation as a rotation matrix (body frame → world frame)
	rot [3][3]float64

	// World-frame velocity (m/s)
	velX, velY, velZ float64

	// World-frame position (m)
	posX, posY, posZ float64

	// Last timestamp (nanoseconds, device clock)
	lastTS uint64
	inited bool
}

var dr = &deadReckoner{}

func init() {
	// Identity rotation
	dr.rot[0][0] = 1
	dr.rot[1][1] = 1
	dr.rot[2][2] = 1
}

const gravityG = 9.80665

// update is called from the IMU callback at 200Hz.
// gyro is in rad/s, acc is in g.
func (d *deadReckoner) update(gyroX, gyroY, gyroZ, accX, accY, accZ float32, tsNs uint64) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if !d.inited {
		d.lastTS = tsNs
		d.inited = true
		// Initialize orientation from first accelerometer reading (gravity direction)
		d.initOrientationFromGravity(accX, accY, accZ)
		return
	}

	dt := float64(tsNs-d.lastTS) / 1e9 // seconds
	d.lastTS = tsNs

	if dt <= 0 || dt > 0.1 { // skip bogus intervals
		return
	}

	// 1. Update orientation by integrating gyroscope (small-angle rotation)
	wx, wy, wz := float64(gyroX)*dt, float64(gyroY)*dt, float64(gyroZ)*dt
	d.applyRotation(wx, wy, wz)

	// 2. Rotate accelerometer reading into world frame
	ax := float64(accX) * gravityG
	ay := float64(accY) * gravityG
	az := float64(accZ) * gravityG

	worldAX := d.rot[0][0]*ax + d.rot[0][1]*ay + d.rot[0][2]*az
	worldAY := d.rot[1][0]*ax + d.rot[1][1]*ay + d.rot[1][2]*az
	worldAZ := d.rot[2][0]*ax + d.rot[2][1]*ay + d.rot[2][2]*az

	// 3. Remove gravity (world Z is up)
	worldAZ -= gravityG

	// 4. Integrate acceleration → velocity → position
	d.velX += worldAX * dt
	d.velY += worldAY * dt
	d.velZ += worldAZ * dt

	d.posX += d.velX * dt
	d.posY += d.velY * dt
	d.posZ += d.velZ * dt
}

// position returns the current estimated position in meters.
func (d *deadReckoner) position() (x, y, z float64) {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.posX, d.posY, d.posZ
}

// orientation returns the current rotation as an orientation vector (axis + angle).
func (d *deadReckoner) orientation() (ox, oy, oz, theta float64) {
	d.mu.Lock()
	defer d.mu.Unlock()

	// Extract angle-axis from rotation matrix
	// angle = arccos((trace(R) - 1) / 2)
	trace := d.rot[0][0] + d.rot[1][1] + d.rot[2][2]
	cosAngle := (trace - 1) / 2
	cosAngle = math.Max(-1, math.Min(1, cosAngle)) // clamp
	theta = math.Acos(cosAngle)

	if theta < 1e-10 {
		return 0, 0, 1, 0 // identity
	}

	// axis = [R32-R23, R13-R31, R21-R12] / (2*sin(theta))
	s := 2 * math.Sin(theta)
	ox = (d.rot[2][1] - d.rot[1][2]) / s
	oy = (d.rot[0][2] - d.rot[2][0]) / s
	oz = (d.rot[1][0] - d.rot[0][1]) / s

	// Convert to degrees for Viam
	theta = theta * (180.0 / math.Pi)
	return ox, oy, oz, theta
}

// initOrientationFromGravity sets the initial rotation matrix so that
// the measured gravity vector aligns with world -Z.
func (d *deadReckoner) initOrientationFromGravity(accX, accY, accZ float32) {
	// Measured gravity in body frame (normalized)
	gx, gy, gz := float64(accX), float64(accY), float64(accZ)
	norm := math.Sqrt(gx*gx + gy*gy + gz*gz)
	if norm < 0.1 {
		return
	}
	gx /= norm
	gy /= norm
	gz /= norm

	// We want to find R such that R * [gx,gy,gz] = [0,0,-1]
	// (gravity in body frame maps to -Z in world frame)
	// Use Rodrigues' rotation formula
	// cross = g × [0,0,-1]
	cx := gy*(-1) - gz*0 // gy*(-1)
	cy := gz*0 - gx*(-1) // gx
	cz := gx*0 - gy*0    // 0

	dot := gx*0 + gy*0 + gz*(-1) // -gz
	crossNorm := math.Sqrt(cx*cx + cy*cy + cz*cz)

	if crossNorm < 1e-10 {
		// Already aligned (or anti-aligned)
		if dot > 0 {
			// Anti-aligned: rotate 180° around X
			d.rot[1][1] = -1
			d.rot[2][2] = -1
		}
		return
	}

	// Normalize cross product (rotation axis)
	cx /= crossNorm
	cy /= crossNorm
	cz /= crossNorm

	angle := math.Atan2(crossNorm, dot)
	d.applyRotation(cx*angle, cy*angle, cz*angle)
}

// applyRotation applies a small rotation (Rodrigues) to the current rotation matrix.
func (d *deadReckoner) applyRotation(wx, wy, wz float64) {
	angle := math.Sqrt(wx*wx + wy*wy + wz*wz)
	if angle < 1e-15 {
		return
	}

	// Normalize axis
	ax, ay, az := wx/angle, wy/angle, wz/angle
	c := math.Cos(angle)
	s := math.Sin(angle)
	t := 1 - c

	// Rodrigues rotation matrix
	var dR [3][3]float64
	dR[0][0] = t*ax*ax + c
	dR[0][1] = t*ax*ay - s*az
	dR[0][2] = t*ax*az + s*ay
	dR[1][0] = t*ay*ax + s*az
	dR[1][1] = t*ay*ay + c
	dR[1][2] = t*ay*az - s*ax
	dR[2][0] = t*az*ax - s*ay
	dR[2][1] = t*az*ay + s*ax
	dR[2][2] = t*az*az + c

	// R_new = dR * R_old
	var newRot [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			for k := 0; k < 3; k++ {
				newRot[i][j] += dR[i][k] * d.rot[k][j]
			}
		}
	}
	d.rot = newRot
}
