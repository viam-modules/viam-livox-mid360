package main

import "go.viam.com/rdk/resource"

var (
	cameraModel = resource.NewModel("viam", "livox", "mid360")
	imuModel    = resource.NewModel("viam", "livox", "mid360-imu")
)

// Config is shared by both the camera and movement sensor components.
type Config struct {
	SensorIP string `json:"sensor_ip"`
	HostIP   string `json:"host_ip,omitempty"`
	TimeSync string `json:"time_sync,omitempty"` // "none", "ptp", "gps"
}

func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.SensorIP == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "sensor_ip")
	}
	return nil, nil, nil
}
