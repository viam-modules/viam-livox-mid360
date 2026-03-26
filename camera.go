package main

import (
	"context"
	"fmt"
	"image/color"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

func init() {
	resource.RegisterComponent(
		camera.API,
		cameraModel,
		resource.Registration[camera.Camera, *Config]{
			Constructor: newMid360Camera,
		},
	)
}

type mid360Camera struct {
	resource.AlwaysRebuild
	name   resource.Name
	logger logging.Logger
}

func newMid360Camera(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (camera.Camera, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	if err := sdkMgr.Acquire(cfg); err != nil {
		return nil, fmt.Errorf("SDK acquire: %w", err)
	}

	return &mid360Camera{
		name:   conf.ResourceName(),
		logger: logger,
	}, nil
}

func (c *mid360Camera) Name() resource.Name {
	return c.name
}

func (c *mid360Camera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	select {
	case <-frames.ready:
	case <-ctx.Done():
		return nil, ctx.Err()
	case <-time.After(2 * time.Second):
		return nil, fmt.Errorf("timeout waiting for point cloud frame")
	}

	frames.mu.Lock()
	pts := make([]pointData, len(frames.reading))
	copy(pts, frames.reading)
	frames.mu.Unlock()

	pc := pointcloud.NewBasicEmpty()
	for _, p := range pts {
		vec := r3.Vector{
			X: float64(p.X),
			Y: float64(p.Y),
			Z: float64(p.Z),
		}
		r := p.Reflectivity
		d := pointcloud.NewColoredData(color.NRGBA{R: r, G: r, B: r, A: 255})
		d.SetValue(int(r))
		if err := pc.Set(vec, d); err != nil {
			c.logger.Debugw("failed to set point", "error", err)
		}
	}

	return pc, nil
}

func (c *mid360Camera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (c *mid360Camera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{}, data.ErrNoCaptureToStore
}

func (c *mid360Camera) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (c *mid360Camera) Close(ctx context.Context) error {
	sdkMgr.Release()
	return nil
}

func (c *mid360Camera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, fmt.Errorf("DoCommand not implemented")
}
