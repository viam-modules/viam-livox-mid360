package main

import (
	"context"
	"fmt"
	"image/color"
	"math"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
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

// getLatestFrame waits for and returns a copy of the latest point data.
func (c *mid360Camera) getLatestFrame(ctx context.Context) ([]pointData, error) {
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
	return pts, nil
}

func (c *mid360Camera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	pts, err := c.getLatestFrame(ctx)
	if err != nil {
		return nil, err
	}

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

// Images returns a depth image projected from the latest point cloud frame.
// The Mid-360 has 360° horizontal FOV and ~59° vertical FOV.
// We use an equirectangular projection to map 3D points onto a 2D depth image.
func (c *mid360Camera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pts, err := c.getLatestFrame(ctx)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	if len(pts) == 0 {
		return nil, resource.ResponseMetadata{}, data.ErrNoCaptureToStore
	}

	// Equirectangular projection: azimuth → x, elevation → y
	const (
		width  = 720 // 0.5° per pixel horizontal
		height = 120 // ~0.5° per pixel vertical
		// Mid-360 vertical FOV: roughly -7° to +52°
		elevMin = -7.0 * math.Pi / 180.0
		elevMax = 52.0 * math.Pi / 180.0
	)
	elevRange := elevMax - elevMin

	dm := rimage.NewEmptyDepthMap(width, height)
	for _, p := range pts {
		x, y, z := float64(p.X), float64(p.Y), float64(p.Z)
		dist := math.Sqrt(x*x + y*y + z*z)
		if dist < 1 { // skip near-zero points
			continue
		}

		azimuth := math.Atan2(y, x)                    // -π to π
		elevation := math.Atan2(z, math.Sqrt(x*x+y*y)) // vertical angle

		// Map to pixel coordinates
		px := int((azimuth + math.Pi) / (2 * math.Pi) * float64(width))
		py := int((elevMax - elevation) / elevRange * float64(height))

		if px < 0 || px >= width || py < 0 || py >= height {
			continue
		}

		d := rimage.Depth(dist)
		// Keep the closest point per pixel
		existing := dm.GetDepth(px, py)
		if existing == 0 || d < existing {
			dm.Set(px, py, d)
		}
	}

	now := time.Now()
	namedImg, err := camera.NamedImageFromImage(dm, "depth", utils.MimeTypeRawDepth, data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	return []camera.NamedImage{namedImg}, resource.ResponseMetadata{CapturedAt: now}, nil
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
