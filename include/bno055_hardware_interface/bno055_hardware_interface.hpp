// Copyright 2026 Aditya Kamath
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BNO055_HARDWARE_INTERFACE__BNO055_HARDWARE_INTERFACE_HPP_
#define BNO055_HARDWARE_INTERFACE__BNO055_HARDWARE_INTERFACE_HPP_

#include <map>
#include <string>
#include <utility>
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
 * in NDOF fusion mode and exposes them as state interfaces for use with
 * imu_sensor_broadcaster.
 *
 * HARDWARE PARAMETERS (from ros2_control URDF <sensor>):
 *   - i2c_bus      : I2C bus number, e.g. "1" for /dev/i2c-1  (default: "1")
 *   - i2c_addr     : Hex I2C address, e.g. "0x28"             (default: "0x28")
 *   - axis_remap   : Placement config P0-P7 (datasheet §3.4)  (default: "P1")
 *   - enable_mock  : Skip real hardware, publish zeros         (default: "false")
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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_;

  // Parameters
  int         i2c_bus_{1};
  uint8_t     i2c_addr_{0x28};
  std::string axis_remap_{"P1"};
  bool        enable_mock_{false};

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
