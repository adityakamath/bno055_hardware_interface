#ifndef BNO055_HARDWARE_INTERFACE__BNO055_HARDWARE_INTERFACE_HPP_
#define BNO055_HARDWARE_INTERFACE__BNO055_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

// Bosch Sensortec driver (C) — pull in with C linkage
#include "bno055_hardware_interface/bno055_i2c.h"

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace bno055_hardware_interface
{

/**
 * @brief ros2_control SensorInterface for the Bosch BNO055 IMU over I2C.
 *
 * Uses the official Bosch Sensortec BNO055 driver (external/BNO055_driver)
 * via a Linux i2c-dev / SMBus adapter (bno055_i2c.c).
 * Reads orientation (quaternion), angular velocity and linear acceleration
 * in a configurable fusion mode (NDOF, NDOF_FMC_OFF, or IMUPLUS) and
 * exposes them as state interfaces for use with imu_sensor_broadcaster.
 *
 * HARDWARE PARAMETERS (from ros2_control URDF <sensor>):
 *   - i2c_bus      : I2C bus number, e.g. "1" for /dev/i2c-1  (default: "1")
 *   - i2c_addr     : Hex I2C address without 0x prefix, e.g. "28"  (default: "28")
 *   - axis_remap   : Placement config P0-P7 (datasheet §3.4)  (default: "P1")
 *   - enable_mock  : Skip real hardware, publish zeros         (default: "false")
 *   - calib_file   : Absolute path to YAML calibration file
 *                    (e.g. /home/user/.ros/bno055_calib.yaml).
 *                    When the file exists offsets are loaded onto the
 *                    sensor during on_configure before entering NDOF mode.
 *                    Leave empty (default) to rely on in-sensor calibration.
 *   - sensor_mode  : Fusion operation mode (default: "NDOF"). One of:
 *                    "NDOF"         — 9-DOF fusion, absolute orientation (uses magnetometer)
 *                    "NDOF_FMC_OFF" — same as NDOF but fast mag calibration disabled
 *                    "IMUPLUS"      — 6-DOF fusion, relative orientation, no magnetometer
 *
 * STATE INTERFACES (sensor_name prefix set in URDF):
 *   orientation.x / .y / .z / .w
 *   angular_velocity.x / .y / .z       (rad/s)
 *   linear_acceleration.x / .y / .z    (m/s^2)
 */
class BNO055HardwareInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BNO055HardwareInterface)

  BNO055HardwareInterface()
  : logger_(rclcpp::get_logger("BNO055HardwareInterface")) {}

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;  // NOLINT(deprecation)

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Load calibration offsets from calib_file_ and write them to the sensor
  // (must be called while the sensor is in CONFIG mode).
  // Returns true if the file was found and applied, false if absent or on error.
  bool load_calib_offsets();

  // Suspend the sensor and close the I2C file descriptor.
  // Called by both on_cleanup and on_shutdown.
  void close_hardware();

  // Parse a boolean hardware parameter; returns default_value if the key is absent.
  // Accepts only "true" — mirrors the xacro $(arg ...) string convention.
  bool parse_bool_param(const std::string & key, bool default_value) const;

  rclcpp::Logger logger_;

  // Parameters
  int         i2c_bus_{1};
  uint8_t     i2c_addr_{0x28};
  std::string axis_remap_{"P1"};
  bool        enable_mock_{false};
  std::string calib_file_{};  // empty = no file, do not attempt to load
  std::string sensor_mode_{"NDOF"};  // fusion operation mode

  // Consecutive read failures before returning ERROR (threshold = 10)
  int consecutive_read_errors_{0};

  // Bosch Sensortec driver handle
  bno055_t sensor_{};

  // State storage -- 10 interfaces
  double hw_orientation_x_{0.0};
  double hw_orientation_y_{0.0};
  double hw_orientation_z_{0.0};
  double hw_orientation_w_{1.0};
  double hw_angular_velocity_x_{0.0};
  double hw_angular_velocity_y_{0.0};
  double hw_angular_velocity_z_{0.0};
  double hw_linear_acceleration_x_{0.0};
  double hw_linear_acceleration_y_{0.0};
  double hw_linear_acceleration_z_{0.0};
};

}  // namespace bno055_hardware_interface

#endif  // BNO055_HARDWARE_INTERFACE__BNO055_HARDWARE_INTERFACE_HPP_
