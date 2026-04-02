# Quick Start Guide

Complete setup and usage instructions for the BNO055 Hardware Interface.

---

## Installation

### 1. Install ROS 2 Dependencies

```bash
sudo apt install \
  ros-kilted-ros2-control \
  ros-kilted-ros2-controllers \
  ros-kilted-imu-sensor-broadcaster \
  ros-kilted-robot-state-publisher \
  ros-kilted-xacro \
  i2c-tools
```

### 2. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/adityakamath/bno055_hardware_interface.git
```

If you forgot `--recurse-submodules`, initialise the Bosch SensorAPI submodule afterwards:

```bash
cd bno055_hardware_interface
git submodule update --init --recursive
```

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select bno055_hardware_interface
source install/setup.bash
```

---

## Hardware Setup

### 1. Wire the BNO055 to I2C

The BNO055 communicates over I2C. On Raspberry Pi 5, I2C bus 1 is exposed on the 40-pin header:

| BNO055 Pin | Pi 5 Header Pin | Description |
|------------|-----------------|-------------|
| VIN / VCC | Pin 1 (3.3V) | Power supply |
| GND | Pin 6 (GND) | Ground |
| SDA | Pin 3 (GPIO2) | I2C data |
| SCL | Pin 5 (GPIO3) | I2C clock |
| ADR | GND or VDD | I2C address select |

- **ADR → GND**: I2C address = 0x28 (default, `i2c_addr:=28`)
- **ADR → VDD**: I2C address = 0x29 (`i2c_addr:=29`)

### 2. Enable I2C on Raspberry Pi

```bash
# Enable I2C if not already active
sudo raspi-config nonint do_i2c 0
```

### 3. Grant I2C Permissions

```bash
# One-time: add user to i2c group (requires logout/login to take effect)
sudo usermod -aG i2c $USER

# Or temporary (no logout required):
sudo chmod 666 /dev/i2c-1
```

### 4. Verify the Sensor is Detected

```bash
i2cdetect -y 1
```

Expected output shows `28` (or `29`) in the address grid:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

If the address does not appear, check wiring and I2C bus permissions before proceeding.

---

## Running the Hardware Interface

### Example 1: Real Hardware (default settings)

```bash
# I2C bus 1, address 0x28, axis remap P1, no calibration file
ros2 launch bno055_hardware_interface bno055.launch.py
```

### Example 2: Real Hardware with Custom Bus and Address

```bash
# I2C bus 3, address 0x29 (ADR pin high)
ros2 launch bno055_hardware_interface bno055.launch.py i2c_bus:=3 i2c_addr:=29
```

### Example 3: Mock Mode (no hardware required)

```bash
# Full ros2_control lifecycle, publishes identity quaternion + zero velocity
ros2 launch bno055_hardware_interface bno055.launch.py enable_mock:=true
```

### Example 4: Real Hardware with Saved Calibration

```bash
# Load magnet/gyro/accel offsets from a previously saved YAML file
ros2 launch bno055_hardware_interface bno055.launch.py \
  calib_file:=/home/ubuntu/.ros/bno055_calib.yaml
```

### Example 5: Different Mounting Orientation

```bash
# P3 mounting (sensor PCB rotated 90° — see design.md axis remap table)
ros2 launch bno055_hardware_interface bno055.launch.py axis_remap:=P3
```

---

## Calibration Workflow

The BNO055 auto-calibrates in NDOF mode, but calibration is lost on power cycle. This workflow saves offsets to a YAML file so the next boot starts pre-calibrated.

### Step 1: Obtain Calibration Offsets

Let the sensor run in NDOF mode until all calibration levels reach 3. Read the offsets from the sensor using the Bosch SensorAPI (e.g. via `i2cget` or a custom script) and write them to a YAML file:

```yaml
# ~/.ros/bno055_calib.yaml
accel_offset_x: 14
accel_offset_y: -8
accel_offset_z: -21
accel_radius: 1000
gyro_offset_x: 0
gyro_offset_y: 1
gyro_offset_z: -1
mag_offset_x: 285
mag_offset_y: -134
mag_offset_z: 478
mag_radius: 857
```

All values are raw 16-bit integers as defined by the Bosch SensorAPI offset structs.

### Step 2: Launch with Saved Calibration

```bash
ros2 launch bno055_hardware_interface bno055.launch.py \
  calib_file:=/home/ubuntu/.ros/bno055_calib.yaml
```

The plugin logs each offset set it applied:

```
[ros2_control_node] Calibration offsets loaded from /home/ubuntu/.ros/bno055_calib.yaml
[ros2_control_node]   Accel offsets: x=14 y=-8 z=-21  radius=1000
[ros2_control_node]   Gyro offsets:  x=0 y=1 z=-1
[ros2_control_node]   Mag offsets:   x=285 y=-134 z=478  radius=857
```

### Calibration Tips

| Sub-sensor | How to calibrate |
|-----------|-----------------|
| **Gyroscope** | Place on a stable surface and keep completely still for ~10 s |
| **Accelerometer** | Hold still in at least 6 different orientations (each face pointing down), ~3 s each |
| **Magnetometer** | Move slowly in a figure-8 pattern in the air, away from metal objects |
| **System** | Reaches 3 only when gyro + accel are both at level 3 |

---

## TF Broadcasting

Enable the optional `imu_tf_broadcaster` relay node to publish the IMU orientation as a TF transform (`world → base_link`). This is useful for visualising orientation in Foxglove Studio or RViz2.

```bash
ros2 launch bno055_hardware_interface bno055.launch.py publish_tf:=true
```

The node subscribes to `/imu_sensor_broadcaster/imu` and publishes a TF transform. Default frame IDs can be overridden in the launch configuration.

### Viewing in Foxglove Studio

1. Open [Foxglove Studio](https://foxglove.dev/download) and connect to `ws://localhost:8765`
2. Add a **3D panel**
3. Add a **TF** frame display — the `base_link` frame will rotate with the IMU orientation
4. Add a **Topic** subscriber for `/imu_sensor_broadcaster/imu` to see the raw message

---

## Launch Arguments

<style>
  .args-table {
    transition: all 0.2s ease;
  }

  .args-table:hover {
    transform: translateY(-2px);
    box-shadow: 0 6px 16px rgba(0,0,0,0.25) !important;
  }
</style>

<table class="args-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="4" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🚀 Launch File Arguments</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Argument</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Default</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Type</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>i2c_bus</code></td>
      <td style="padding: 0.6em; border: none;"><code>1</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;">I2C bus number; plugin opens <code>/dev/i2c-{n}</code></td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>i2c_addr</code></td>
      <td style="padding: 0.6em; border: none;"><code>28</code></td>
      <td style="padding: 0.6em; border: none;">string</td>
      <td style="padding: 0.6em; border: none;">Hex address without <code>0x</code>; <code>28</code> = ADR pin low (0x28), <code>29</code> = ADR pin high (0x29)</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>axis_remap</code></td>
      <td style="padding: 0.6em; border: none;"><code>P1</code></td>
      <td style="padding: 0.6em; border: none;">string</td>
      <td style="padding: 0.6em; border: none;">Mounting orientation P0–P7 per BNO055 datasheet §3.4 (see <a href="design.md">design.md</a>)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>enable_mock</code></td>
      <td style="padding: 0.6em; border: none;"><code>false</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;">Skip I2C; publish identity quaternion and zero velocity/acceleration</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>calib_file</code></td>
      <td style="padding: 0.6em; border: none;"><code>""</code></td>
      <td style="padding: 0.6em; border: none;">string</td>
      <td style="padding: 0.6em; border: none;">Absolute path to YAML calibration file; empty = start uncalibrated</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>publish_tf</code></td>
      <td style="padding: 0.6em; border: none;"><code>false</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;">Enable the <code>imu_tf_broadcaster</code> relay node to publish <code>world → base_link</code> TF</td>
    </tr>
  </tbody>
</table>

---

## Testing and Monitoring

### 1. Verify Hardware Connection

```bash
ros2 control list_hardware_interfaces
```

Expected output lists all 10 state interfaces with `[available]` and `[claimed]` status.

### 2. Monitor IMU Data

```bash
# Echo the IMU message
ros2 topic echo /imu_sensor_broadcaster/imu

# Check publish rate
ros2 topic hz /imu_sensor_broadcaster/imu
# Expected: ~100.0 Hz
```

### 3. Check Controller Status

```bash
ros2 control list_controllers
```

Expected output:

```
imu_sensor_broadcaster[imu_sensor_broadcaster/IMUSensorBroadcaster] active
```

### 4. Check Active Topics

```bash
ros2 topic list
```

Expected topics when running:

```
/imu_sensor_broadcaster/imu
/robot_description
/joint_states
/tf
/tf_static
```

### 5. View TF Frames (when publish_tf:=true)

```bash
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### Sensor Not Detected

**Problem:** `i2cdetect` does not show the BNO055 address, or the hardware interface fails to start.

**Solutions:**

1. **Check I2C permissions:**
   ```bash
   sudo chmod 666 /dev/i2c-1
   ```

2. **Verify wiring:**
   - Ensure SDA and SCL are not swapped
   - Check that VCC is 3.3V (not 5V, unless the breakout has a regulator)
   - Confirm the ADR pin matches the configured `i2c_addr`

3. **Test in mock mode:**
   ```bash
   ros2 launch bno055_hardware_interface bno055.launch.py enable_mock:=true
   ```
   If mock mode works, the issue is hardware/I2C related.

### Read Errors During Operation

**Problem:** Frequent "BNO055 read error" warnings in the log.

**Solutions:**

1. **Check cable length and quality:**
   - I2C is designed for short-distance communication (< 1 m)
   - Use shielded cables in electrically noisy environments

2. **Reduce controller update rate:**
   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 50  # Reduce from 100 Hz
   ```

3. **Verify I2C bus speed:**
   - The BNO055 supports up to 400 kHz (Fast Mode)
   - On Pi 5, the default is 100 kHz — this is sufficient

### Calibration File Not Loading

**Problem:** "Calibration file not found or unreadable" warning when launching with `calib_file`.

**Solutions:**

1. **Use absolute paths:**
   ```bash
   calib_file:=/home/ubuntu/.ros/bno055_calib.yaml  # Correct
   calib_file:=~/.ros/bno055_calib.yaml              # Will not work
   ```

2. **Check file permissions:**
   ```bash
   ls -la /home/ubuntu/.ros/bno055_calib.yaml
   ```

3. **Verify YAML format:**
   Each line should be `key: value` format (see [config/bno055_calib.yaml](../config/bno055_calib.yaml) for a template).

### Orientation Drift or Incorrect Heading

**Problem:** The orientation quaternion drifts or the heading is incorrect.

**Solutions:**

1. **Ensure full calibration:**
   All four sub-sensors (gyro, accel, mag, sys) should reach level 3 before saving offsets.

2. **Check for magnetic interference:**
   - Keep the sensor away from motors, speakers, and metal objects
   - The magnetometer is very sensitive to nearby ferrous materials

3. **Verify axis remap:**
   - Confirm the `axis_remap` parameter matches the physical mounting orientation
   - See the [Design document](design.md) for the P0–P7 axis remap table

---

## Running the Test Suite

The package includes a full test suite that runs without physical hardware — no I2C bus or sensor needed.

### Run All Tests

```bash
# Build first (required before running tests)
colcon build --packages-select bno055_hardware_interface

# Run all tests
colcon test --packages-select bno055_hardware_interface

# View detailed pass/fail output
colcon test-result --verbose
```

### Test Groups

| Test File | Type | Tests | What Is Covered |
|-----------|------|-------|-----------------|
| `test_hardware_interface.cpp` | C++ unit | 8 | Parameter parsing, state interface export, mock-mode lifecycle, read behaviour |
| `test_bno055.launch.py` | Launch integration | 8 | Full bringup with `enable_mock:=true`, node presence, topic availability |
| Linters | Style | 4+ | `flake8`, `pep257` (C++ linters disabled to avoid recursing into the Bosch vendor tree) |

### Run Specific Test Groups

```bash
# C++ unit tests only (fast, no ROS nodes required)
colcon test --packages-select bno055_hardware_interface \
  --ctest-args -R "test_hardware_interface"

# Launch integration tests only
colcon test --packages-select bno055_hardware_interface \
  --ctest-args -R "test_bno055"
```

### Interpreting Results

```
Summary: X tests, 0 errors, 0 failures, Y skipped
```

All tests must pass. If any fail, run `colcon test-result --verbose` and check the per-test logs in `log/latest_test/`.

---

## Next Steps

### For Basic Users

- Test with mock mode first, then connect real hardware
- Save calibration offsets after initial sensor setup
- Use Foxglove Studio or RViz2 to visualise orientation in real-time

### For Advanced Users

- Configure axis remapping (P0–P7) for your specific mounting orientation
- Create a custom URDF with the sensor integrated into your robot model
- See the [Design document](design.md) for the on_configure sequence and I2C internals

### For Developers

- Enable mock mode for hardware-free development
- Study the test files for examples of lifecycle and parameter validation
- Review the [Design document](design.md) for system design details

---

## Additional Resources

- [bno055_hardware_interface README](https://github.com/adityakamath/bno055_hardware_interface/blob/main/README.md)
- [Design documentation](design.md)
- [ros2_control documentation](https://control.ros.org/)
- [Bosch BNO055 SensorAPI](https://github.com/BoschSensortec/BNO055_SensorAPI)
- [Bosch BNO055 datasheet](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/)
