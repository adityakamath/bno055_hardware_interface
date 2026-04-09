# BNO055 Hardware Interface

![Project Status](https://img.shields.io/badge/Status-Active-green)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![ROS 2 Control](https://img.shields.io/badge/ros2__control-SensorInterface-blue?style=flat&logo=ros&logoSize=auto)
![Repository](https://img.shields.io/badge/Repo-adityakamath%2Fbno055__hardware__interface-purple?style=flat&logo=github&logoSize=auto)
![Dependency](https://img.shields.io/badge/Dep-BoschSensortec%2FBNO055__driver-purple?style=flat&logo=github&logoSize=auto)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/bno055_hardware_interface)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![License](https://img.shields.io/github/license/adityakamath/bno055_hardware_interface?label=License)

`ros2_control` `SensorInterface` plugin for the Bosch BNO055 9-DOF IMU over I2C (Linux `i2c-dev`).

**⚠️ Status:** Tested and validated on Raspberry Pi 5 running ROS 2 Kilted (Ubuntu 24.04, aarch64) with real BNO055 hardware.

## Features

- **Configurable Sensor Fusion**: Three on-chip fusion modes — `NDOF` (9-DOF absolute, default), `NDOF_FMC_OFF` (9-DOF without fast magnetometer calibration, for magnetically noisy environments), and `IMUPLUS` (6-DOF gyro + accel only, no magnetometer) — selected at launch via `sensor_mode`
- **10 State Interfaces**: Orientation quaternion (x, y, z, w), angular velocity (rad/s), and linear acceleration (m/s²) — fully compatible with `imu_sensor_broadcaster`
- **Bosch SensorAPI Integration**: Uses the official [Bosch BNO055 driver](https://github.com/BoschSensortec/BNO055_driver) C library (git submodule); no third-party wrappers
- **Axis Remapping**: 8 standard mounting orientations (P0–P7) configurable at launch, matching BNO055 datasheet §3.4
- **Calibration Persistence**: Save sensor calibration offsets to a YAML file and load them automatically at boot via the `calib_file` hardware parameter; offsets are written in CONFIG mode before entering NDOF
- **Mock Mode**: Run the complete `ros2_control` lifecycle and publish zero/identity values without any hardware — set `enable_mock:=true`
- **TF Broadcasting**: Optional `imu_tf_broadcaster` relay node republishes the orientation quaternion as a dynamic `world → base_link` TF transform

## Quick Start

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/adityakamath/bno055_hardware_interface.git
cd ~/ros2_ws
colcon build --packages-select bno055_hardware_interface
source install/setup.bash
```

See the **[Quick Start guide](docs/quick-start.md)** for detailed instructions on wiring, calibration, launch arguments, and monitoring.

## Configuration Example

```xml
<ros2_control name="bno055_sensor" type="sensor">
  <hardware>
    <plugin>bno055_hardware_interface/BNO055HardwareInterface</plugin>
    <param name="i2c_bus">1</param>
    <param name="i2c_addr">28</param>
    <param name="axis_remap">P1</param>
    <param name="sensor_mode">NDOF</param>
    <param name="enable_mock">false</param>
    <param name="calib_file"></param>
  </hardware>
  <sensor name="bno055">
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
    <state_interface name="angular_velocity.x"/>
    <state_interface name="angular_velocity.y"/>
    <state_interface name="angular_velocity.z"/>
    <state_interface name="linear_acceleration.x"/>
    <state_interface name="linear_acceleration.y"/>
    <state_interface name="linear_acceleration.z"/>
  </sensor>
</ros2_control>
```

## Hardware Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `i2c_bus` | `int` | `1` | I2C bus number — plugin opens `/dev/i2c-{n}` |
| `i2c_addr` | `string` | `"28"` | I2C address as hex without `0x` prefix (`28` = 0x28 ADR-low, `29` = 0x29 ADR-high) |
| `axis_remap` | `string` | `"P1"` | Mounting orientation P0–P7 per BNO055 datasheet §3.4 |
| `sensor_mode` | `string` | `"NDOF"` | Fusion mode: `NDOF` (9-DOF absolute), `NDOF_FMC_OFF` (9-DOF, magnetically noisy env), `IMUPLUS` (6-DOF, no magnetometer). Invalid values cause `on_init` to return `ERROR` |
| `enable_mock` | `bool` | `false` | Skip I2C initialisation; publish constant zero/identity values |
| `calib_file` | `string` | `""` | Absolute path to calibration YAML; empty = start uncalibrated |

## State Interfaces

| Interface | Unit | Notes |
|-----------|------|-------|
| `orientation.x` | – | Quaternion X — raw reading ÷ 16384 (2¹⁴, Bosch datasheet §3.6.5.5) |
| `orientation.y` | – | Quaternion Y |
| `orientation.z` | – | Quaternion Z |
| `orientation.w` | – | Quaternion W |
| `angular_velocity.x` | rad/s | Gyroscope unit set to rad/s at configure time |
| `angular_velocity.y` | rad/s | Gyroscope unit set to rad/s at configure time |
| `angular_velocity.z` | rad/s | Gyroscope unit set to rad/s at configure time |
| `linear_acceleration.x` | m/s² | Accelerometer unit set to m/s² at configure time |
| `linear_acceleration.y` | m/s² | Accelerometer unit set to m/s² at configure time |
| `linear_acceleration.z` | m/s² | Accelerometer unit set to m/s² at configure time |

## Documentation

- **[Quick Start guide](docs/quick-start.md)** — Installation, hardware wiring, calibration workflow, launch args, monitoring
- **[Design documentation](docs/design.md)** — Plugin architecture, NDOF fusion, on_configure sequence, axis remap table, calibration internals
