package main

import (
	"math"
	"testing"
)

func TestOrientationAxisAngle(t *testing.T) {
	d := &deadReckoner{}
	d.rot[0][0] = 1
	d.rot[1][1] = 1
	d.rot[2][2] = 1

	// Identity
	aa := d.orientation()
	if aa.Theta > 1e-9 {
		t.Fatalf("expected identity, got theta=%v", aa.Theta)
	}

	// +90° about Z, applied as two 45° increments
	d.applyRotation(0, 0, math.Pi/4)
	d.applyRotation(0, 0, math.Pi/4)
	aa = d.orientation()
	if math.Abs(aa.Theta-math.Pi/2) > 1e-9 {
		t.Errorf("theta = %v, want pi/2", aa.Theta)
	}
	if math.Abs(aa.RZ-1) > 1e-9 || math.Abs(aa.RX) > 1e-9 || math.Abs(aa.RY) > 1e-9 {
		t.Errorf("axis = (%v, %v, %v), want (0, 0, 1)", aa.RX, aa.RY, aa.RZ)
	}

	// -30° about X on a fresh reckoner
	d = &deadReckoner{}
	d.rot[0][0] = 1
	d.rot[1][1] = 1
	d.rot[2][2] = 1
	d.applyRotation(-math.Pi/6, 0, 0)
	aa = d.orientation()
	if math.Abs(aa.Theta-math.Pi/6) > 1e-9 {
		t.Errorf("theta = %v, want pi/6", aa.Theta)
	}
	if math.Abs(aa.RX+1) > 1e-9 {
		t.Errorf("axis = (%v, %v, %v), want (-1, 0, 0)", aa.RX, aa.RY, aa.RZ)
	}
}

func TestInitOrientationFromGravity(t *testing.T) {
	cases := []struct {
		name       string
		ax, ay, az float32 // at-rest accelerometer reading in g (unit norm)
	}{
		{"level", 0, 0, 1},
		{"upside-down", 0, 0, -1},
		{"pitched-30", 0.5, 0, 0.8660254},
		{"rolled-45", 0, 0.70710678, 0.70710678},
		{"arbitrary", 0.36, 0.48, 0.8},
	}
	for _, c := range cases {
		d := &deadReckoner{}
		d.rot[0][0] = 1
		d.rot[1][1] = 1
		d.rot[2][2] = 1
		d.initOrientationFromGravity(c.ax, c.ay, c.az)

		// The measured up-vector must map to world +Z so that update()'s
		// gravity subtraction cancels at rest.
		ux, uy, uz := float64(c.ax), float64(c.ay), float64(c.az)
		wx := d.rot[0][0]*ux + d.rot[0][1]*uy + d.rot[0][2]*uz
		wy := d.rot[1][0]*ux + d.rot[1][1]*uy + d.rot[1][2]*uz
		wz := d.rot[2][0]*ux + d.rot[2][1]*uy + d.rot[2][2]*uz
		if math.Abs(wx) > 1e-7 || math.Abs(wy) > 1e-7 || math.Abs(wz-1) > 1e-7 {
			t.Errorf("%s: R*u = (%v, %v, %v), want (0, 0, 1)", c.name, wx, wy, wz)
		}
	}
}
