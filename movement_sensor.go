package main

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

func init() {
	resource.RegisterComponent(
		movementsensor.API,
		imuModel,
		resource.Registration[movementsensor.MovementSensor, *Config]{
			Constructor: newMid360IMU,
		},
	)
}

type mid360IMU struct {
	resource.Named
	resource.AlwaysRebuild
	logger logging.Logger
}

func newMid360IMU(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (movementsensor.MovementSensor, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	if err := sdkMgr.Acquire(cfg); err != nil {
		return nil, fmt.Errorf("SDK acquire: %w", err)
	}

	return &mid360IMU{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}, nil
}

func (m *mid360IMU) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	v := latestIMU.Load()
	if v == nil {
		return spatialmath.AngularVelocity{}, fmt.Errorf("no IMU data yet")
	}
	imu := v.(*imuReading)
	// Livox SDK reports gyro in rad/s; Viam GetAngularVelocity is deg/s.
	return spatialmath.AngularVelocity{
		X: float64(imu.GyroX) * (180.0 / math.Pi),
		Y: float64(imu.GyroY) * (180.0 / math.Pi),
		Z: float64(imu.GyroZ) * (180.0 / math.Pi),
	}, nil
}

func (m *mid360IMU) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	v := latestIMU.Load()
	if v == nil {
		return r3.Vector{}, fmt.Errorf("no IMU data yet")
	}
	imu := v.(*imuReading)
	const g = 9.80665
	return r3.Vector{
		X: float64(imu.AccX) * g,
		Y: float64(imu.AccY) * g,
		Z: float64(imu.AccZ) * g,
	}, nil
}

func (m *mid360IMU) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	return r3.Vector{}, movementsensor.ErrMethodUnimplementedLinearVelocity
}

func (m *mid360IMU) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	return nil, 0, movementsensor.ErrMethodUnimplementedPosition
}

func (m *mid360IMU) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	return 0, movementsensor.ErrMethodUnimplementedCompassHeading
}

func (m *mid360IMU) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	return nil, movementsensor.ErrMethodUnimplementedOrientation
}

func (m *mid360IMU) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	return &movementsensor.Properties{
		AngularVelocitySupported:    true,
		LinearAccelerationSupported: true,
	}, nil
}

func (m *mid360IMU) Accuracy(ctx context.Context, extra map[string]interface{}) (*movementsensor.Accuracy, error) {
	return movementsensor.UnimplementedOptionalAccuracies(), nil
}

func (m *mid360IMU) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	readings := make(map[string]interface{})

	av, err := m.AngularVelocity(ctx, extra)
	if err == nil {
		readings["angular_velocity"] = av
	}

	la, err := m.LinearAcceleration(ctx, extra)
	if err == nil {
		readings["linear_acceleration"] = la
	}

	v := latestIMU.Load()
	if v != nil {
		imu := v.(*imuReading)
		readings["timestamp_ns"] = imu.Timestamp
	}

	return readings, nil
}

func (m *mid360IMU) Close(ctx context.Context) error {
	sdkMgr.Release()
	return nil
}
