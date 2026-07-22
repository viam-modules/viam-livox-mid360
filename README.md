# [`viam-livox-mid360` module](https://github.com/viam-modules/viam-livox-mid360)

This [Livox Mid-360 module](https://app.viam.com/module/viam/livox-mid360) provides a driver for the
[Livox Mid-360 LiDAR](https://www.livoxtech.com/mid-360), exposing two components:

- a [`rdk:component:camera` API](https://docs.viam.com/appendix/apis/components/camera/) that serves the point cloud (`viam:livox:mid360`), and
- a [`rdk:component:movement_sensor` API](https://docs.viam.com/appendix/apis/components/movement-sensor/) that serves the built-in IMU (`viam:livox:mid360-imu`).

> [!NOTE]
> Before configuring your components, you must [create a machine](https://docs.viam.com/cloud/machines/#add-a-new-machine).

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in the [Viam app](https://app.viam.com/).
Add camera / `viam:livox:mid360` and/or movement_sensor / `viam:livox:mid360-imu` to your machine.

## Configure your Mid-360 camera

Copy and paste the following attribute template into the `viam:livox:mid360` component's attributes field:

```json
{
  "sensor_ip": "192.168.1.1xx",
  "host_ip": "",
  "time_sync": "none"
}
```

## Configure your Mid-360 IMU

Copy and paste the following attribute template into the `viam:livox:mid360-imu` component's attributes field:

```json
{
  "sensor_ip": "192.168.1.1xx",
  "host_ip": "",
  "time_sync": "none"
}
```

### Attributes

The following attributes are available for both `viam:livox:mid360` and `viam:livox:mid360-imu`:

| Name        | Type   | Inclusion    | Default | Description                                                                                              |
|-------------|--------|--------------|---------|----------------------------------------------------------------------------------------------------------|
| `sensor_ip` | string | **Required** | -       | The IP address of the Mid-360 LiDAR on your network.                                                     |
| `host_ip`   | string | Optional     | ""      | The IP address of the host interface that receives sensor data. Leave empty to auto-select.              |
| `time_sync` | string | Optional     | `none`  | Time synchronization source: `none` (device clock, relative), `ptp` (PTP master on network), or `gps`.   |

## Components

### Camera (`viam:livox:mid360`)

Implements `NextPointCloud()` and `Properties()`. `NextPointCloud()` returns one complete frame of
accumulated points (~20k points per frame at 10 Hz). Point coordinates are returned in meters, with
reflectivity carried as the point value. `Images()` is not implemented.

### MovementSensor (`viam:livox:mid360-imu`)

Serves the built-in IMU. `AngularVelocity()` returns gyro readings (rad/s) and `LinearAcceleration()`
returns accelerometer readings. `Position`, `LinearVelocity`, `CompassHeading`, and `Orientation` are
not supported and return the appropriate unimplemented errors.

## Next Steps

- To test your components, expand the **TEST** section of the configuration pane or go to the [**CONTROL** tab](https://docs.viam.com/fleet/control/).
- To write code against these components, use one of the [available SDKs](https://docs.viam.com/sdks/).
