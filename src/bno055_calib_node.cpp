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

/** @file bno055_calib_node.cpp
 *
 * Standalone ROS 2 node for monitoring and managing BNO055 IMU calibration.
 *
 * The BNO055 runs in NDOF (9-DOF sensor fusion) mode and calibrates itself
 * over time.  This node:
 *   1. Prints calibration status and movement guidance to the terminal at 1 Hz.
 *   2. Publishes a diagnostic_msgs/DiagnosticArray on ~/diagnostics at 1 Hz
 *      so external tools (e.g. rqt_runtime_monitor) can visualise the status.
 *   3. Provides two services to persist calibration offsets to a YAML file so
 *      that re-calibration after every boot is not required.
 *
 * Topics published:
 *   ~/diagnostics  (diagnostic_msgs/DiagnosticArray, 1 Hz)
 *
 * Services:
 *   ~/save_calibration  (std_srvs/Trigger)
 *       Switches sensor to CONFIG mode, reads accel/gyro/mag offsets and
 *       radii, and writes them to the calibration file.
 *
 *   ~/load_calibration  (std_srvs/Trigger)
 *       Reads accel/gyro/mag offsets from the calibration file, switches
 *       sensor to CONFIG mode, writes the offsets to the sensor, then
 *       returns to NDOF mode.  Call this on startup to skip re-calibration.
 *
 * Parameters:
 *   i2c_bus   (int,    default 1)
 *       I2C bus number — node opens /dev/i2c-<i2c_bus>.
 *   i2c_addr  (int,    default 0x28)
 *       I2C device address (0x28 or 0x29 depending on ADR pin).
 *   calib_file (string, default "~/.ros/bno055_calib.yaml")
 *       Path to the calibration profile YAML file.
 *
 * Calibration tips printed at runtime:
 *   Gyroscope    (0-3) — place the sensor on a stable surface and keep still.
 *   Accelerometer(0-3) — hold the sensor still in at least 6 different
 *                        positions (e.g. each face pointing down).
 *   Magnetometer (0-3) — move the sensor in a slow figure-8 pattern in the air.
 *   System       (0-3) — reflects the overall fusion quality; reaches 3 only
 *                        when gyro+accel are fully calibrated.
 */

#include <fcntl.h>

#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

extern "C" {
#include "bno055_hardware_interface/bno055_i2c.h"
}

namespace bno055_hardware_interface
{

using DiagArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue = diagnostic_msgs::msg::KeyValue;
using Trigger = std_srvs::srv::Trigger;

// ── small helpers ─────────────────────────────────────────────────────────────

/// ASCII progress bar: 0 → "[    ]", 3 → "[*** ]", 4 → "[****]"
static std::string bar(u8 v)
{
  std::string s(4, ' ');
  for (u8 i = 0; i < v && i < 4; ++i) {s[i] = '*';}
  return s;
}

static uint8_t diag_level(u8 v)
{
  if (v >= 3) {return DiagStatus::OK;}
  if (v >= 1) {return DiagStatus::WARN;}
  return DiagStatus::ERROR;
}

static KeyValue make_kv(const std::string & key, const std::string & val)
{
  KeyValue kv;
  kv.key = key;
  kv.value = val;
  return kv;
}

static std::string expand_home(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) {return std::string(home) + path.substr(1);}
  }
  return path;
}

// ── node ──────────────────────────────────────────────────────────────────────

class BNO055CalibNode : public rclcpp::Node
{
public:
  explicit BNO055CalibNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("bno055_calib_node", options)
  {
    declare_parameter("i2c_bus",   1);
    declare_parameter("i2c_addr",  0x28);
    declare_parameter("calib_file", expand_home("~/.ros/bno055_calib.yaml"));

    i2c_bus_    = get_parameter("i2c_bus").as_int();
    i2c_addr_   = static_cast<u8>(get_parameter("i2c_addr").as_int());
    calib_file_ = get_parameter("calib_file").as_string();

    RCLCPP_INFO(
      get_logger(),
      "BNO055 calibration node: i2c_bus=%d addr=0x%02X  calib_file=%s",
      i2c_bus_, i2c_addr_, calib_file_.c_str());

    diag_pub_ = create_publisher<DiagArray>("~/diagnostics", 10);

    save_srv_ = create_service<Trigger>(
      "~/save_calibration",
      std::bind(
        &BNO055CalibNode::save_cb, this,
        std::placeholders::_1, std::placeholders::_2));

    load_srv_ = create_service<Trigger>(
      "~/load_calibration",
      std::bind(
        &BNO055CalibNode::load_cb, this,
        std::placeholders::_1, std::placeholders::_2));

    if (!init_sensor()) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to initialize BNO055 — check wiring and I2C address.");
      throw std::runtime_error("BNO055 init failed");
    }

    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "Calibration guidance:");
    RCLCPP_INFO(get_logger(), "  Gyroscope     — place sensor on a stable surface and keep still.");
    RCLCPP_INFO(
      get_logger(),
      "  Accelerometer — hold sensor still in >=6 different stationary positions.");
    RCLCPP_INFO(
      get_logger(),
      "  Magnetometer  — move sensor slowly in a figure-8 pattern in the air.");
    RCLCPP_INFO(get_logger(), "");

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BNO055CalibNode::on_timer, this));
  }

  ~BNO055CalibNode() override
  {
    if (sensor_ready_) {
      bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
      bno055_i2c_close();
    }
  }

private:
  // ── sensor init ─────────────────────────────────────────────────────────────

  bool init_sensor()
  {
    std::string dev = "/dev/i2c-" + std::to_string(i2c_bus_);
    if (bno055_i2c_open(dev.c_str(), i2c_addr_) != 0) {
      RCLCPP_ERROR(
        get_logger(), "Cannot open %s at 0x%02X", dev.c_str(), i2c_addr_);
      return false;
    }

    sensor_.bus_read   = BNO055_I2C_bus_read;
    sensor_.bus_write  = BNO055_I2C_bus_write;
    sensor_.delay_msec = BNO055_delay_msek;
    sensor_.dev_addr   = i2c_addr_;

    if (bno055_init(&sensor_) != BNO055_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "bno055_init() failed — check chip wiring");
      bno055_i2c_close();
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "BNO055 detected: chip_id=0x%02X", sensor_.chip_id);

    // Switch to CONFIG mode before applying settings
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    BNO055_delay_msek(25);
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    BNO055_delay_msek(10);

    // NDOF mode enables all three sensors + sensor fusion
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    BNO055_delay_msek(20);

    sensor_ready_ = true;
    RCLCPP_INFO(get_logger(), "Running in NDOF fusion mode — monitoring calibration...");
    return true;
  }

  // ── 1 Hz timer ──────────────────────────────────────────────────────────────

  void on_timer()
  {
    u8 sys = 0, gyro = 0, accel = 0, mag = 0;
    bno055_get_sys_calib_stat(&sys);
    bno055_get_gyro_calib_stat(&gyro);
    bno055_get_accel_calib_stat(&accel);
    bno055_get_mag_calib_stat(&mag);

    RCLCPP_INFO(get_logger(), "──────────────────────────────────────────────");
    RCLCPP_INFO(get_logger(), "  System:        [%s] %d/3  %s",
      bar(sys).c_str(), sys,
      sys == 3 ? "CALIBRATED" : "waiting for sub-sensors");
    RCLCPP_INFO(get_logger(), "  Gyroscope:     [%s] %d/3  %s",
      bar(gyro).c_str(), gyro,
      gyro == 3 ? "OK" : "keep sensor still");
    RCLCPP_INFO(get_logger(), "  Accelerometer: [%s] %d/3  %s",
      bar(accel).c_str(), accel,
      accel == 3 ? "OK" : "tilt into >=6 different stationary positions");
    RCLCPP_INFO(get_logger(), "  Magnetometer:  [%s] %d/3  %s",
      bar(mag).c_str(), mag,
      mag == 3 ? "OK" : "move in a figure-8 pattern");

    if (sys == 3 && gyro == 3 && accel == 3 && mag == 3) {
      RCLCPP_INFO(
        get_logger(),
        "  >>> Fully calibrated! Call ~/save_calibration to persist offsets.");
    }

    // Build and publish DiagnosticArray
    DiagArray msg;
    msg.header.stamp = get_clock()->now();

    auto add = [&](const std::string & name, u8 v, const std::string & hint) {
        DiagStatus s;
        s.hardware_id = "bno055";
        s.name = "BNO055 " + name + " Calibration";
        s.level = diag_level(v);
        s.message = (v == 3)
          ? "Fully calibrated (3/3)"
          : hint + " (" + std::to_string(v) + "/3)";
        s.values.push_back(make_kv("status", std::to_string(v)));
        s.values.push_back(make_kv("bar", "[" + bar(v) + "]"));
        msg.status.push_back(s);
      };

    add("System",        sys,   "Waiting for sub-sensors");
    add("Gyroscope",     gyro,  "Keep sensor still");
    add("Accelerometer", accel, "Tilt into >=6 stationary positions");
    add("Magnetometer",  mag,   "Move in figure-8 pattern");

    diag_pub_->publish(msg);
  }

  // ── save_calibration ────────────────────────────────────────────────────────

  void save_cb(
    const std::shared_ptr<Trigger::Request> /*req*/,
    std::shared_ptr<Trigger::Response> resp)
  {
    RCLCPP_INFO(get_logger(), "save_calibration: switching to CONFIG mode...");

    // Offset registers are only accessible in CONFIG mode
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    BNO055_delay_msek(25);

    bno055_accel_offset_t ao{};
    bno055_gyro_offset_t  go{};
    bno055_mag_offset_t   mo{};

    bool ok =
      (bno055_read_accel_offset(&ao) == BNO055_SUCCESS) &&
      (bno055_read_gyro_offset(&go)  == BNO055_SUCCESS) &&
      (bno055_read_mag_offset(&mo)   == BNO055_SUCCESS);

    // Always return to NDOF regardless of read success
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    BNO055_delay_msek(20);

    if (!ok) {
      resp->success = false;
      resp->message = "Failed to read calibration offsets from sensor";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    if (!write_calib_file(ao, go, mo)) {
      resp->success = false;
      resp->message = "Failed to write calibration file: " + calib_file_;
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    resp->success = true;
    resp->message = "Calibration saved to " + calib_file_;
    RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    RCLCPP_INFO(
      get_logger(),
      "  Accel: x=%d y=%d z=%d  radius=%d", ao.x, ao.y, ao.z, ao.r);
    RCLCPP_INFO(
      get_logger(),
      "  Gyro:  x=%d y=%d z=%d", go.x, go.y, go.z);
    RCLCPP_INFO(
      get_logger(),
      "  Mag:   x=%d y=%d z=%d  radius=%d", mo.x, mo.y, mo.z, mo.r);
  }

  // ── load_calibration ────────────────────────────────────────────────────────

  void load_cb(
    const std::shared_ptr<Trigger::Request> /*req*/,
    std::shared_ptr<Trigger::Response> resp)
  {
    RCLCPP_INFO(
      get_logger(), "load_calibration: reading from %s ...", calib_file_.c_str());

    bno055_accel_offset_t ao{};
    bno055_gyro_offset_t  go{};
    bno055_mag_offset_t   mo{};

    if (!read_calib_file(ao, go, mo)) {
      resp->success = false;
      resp->message = "Failed to read calibration file: " + calib_file_;
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Switching to CONFIG mode to apply offsets...");

    // Offset registers are only writable in CONFIG mode
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    BNO055_delay_msek(25);

    bool ok =
      (bno055_write_accel_offset(&ao) == BNO055_SUCCESS) &&
      (bno055_write_gyro_offset(&go)  == BNO055_SUCCESS) &&
      (bno055_write_mag_offset(&mo)   == BNO055_SUCCESS);

    // Return to fusion mode regardless of write success
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    BNO055_delay_msek(20);

    if (!ok) {
      resp->success = false;
      resp->message = "Failed to write calibration offsets to sensor";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    resp->success = true;
    resp->message = "Calibration loaded from " + calib_file_;
    RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    RCLCPP_INFO(
      get_logger(),
      "  Accel: x=%d y=%d z=%d  radius=%d", ao.x, ao.y, ao.z, ao.r);
    RCLCPP_INFO(
      get_logger(),
      "  Gyro:  x=%d y=%d z=%d", go.x, go.y, go.z);
    RCLCPP_INFO(
      get_logger(),
      "  Mag:   x=%d y=%d z=%d  radius=%d", mo.x, mo.y, mo.z, mo.r);
  }

  // ── file I/O ────────────────────────────────────────────────────────────────

  bool write_calib_file(
    const bno055_accel_offset_t & ao,
    const bno055_gyro_offset_t & go,
    const bno055_mag_offset_t & mo)
  {
    std::ofstream f(calib_file_);
    if (!f) {return false;}
    f << "# BNO055 calibration profile\n";
    f << "# Written by bno055_calib_node. Do not edit manually.\n";
    f << "accel_offset_x: " << ao.x << "\n";
    f << "accel_offset_y: " << ao.y << "\n";
    f << "accel_offset_z: " << ao.z << "\n";
    f << "accel_radius:   " << ao.r << "\n";
    f << "gyro_offset_x:  " << go.x << "\n";
    f << "gyro_offset_y:  " << go.y << "\n";
    f << "gyro_offset_z:  " << go.z << "\n";
    f << "mag_offset_x:   " << mo.x << "\n";
    f << "mag_offset_y:   " << mo.y << "\n";
    f << "mag_offset_z:   " << mo.z << "\n";
    f << "mag_radius:     " << mo.r << "\n";
    return f.good();
  }

  bool read_calib_file(
    bno055_accel_offset_t & ao,
    bno055_gyro_offset_t & go,
    bno055_mag_offset_t & mo)
  {
    std::ifstream f(calib_file_);
    if (!f) {return false;}

    std::unordered_map<std::string, int16_t> vals;
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      auto colon = line.find(':');
      if (colon == std::string::npos) {continue;}
      std::string key = line.substr(0, colon);
      // trim trailing whitespace from key
      while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) {
        key.pop_back();
      }
      std::string val_str = line.substr(colon + 1);
      try {
        vals[key] = static_cast<int16_t>(std::stoi(val_str));
      } catch (...) {
        continue;
      }
    }

    auto get = [&](const std::string & k) -> int16_t {
        auto it = vals.find(k);
        return (it != vals.end()) ? it->second : int16_t(0);
      };

    ao.x = get("accel_offset_x");
    ao.y = get("accel_offset_y");
    ao.z = get("accel_offset_z");
    ao.r = get("accel_radius");
    go.x = get("gyro_offset_x");
    go.y = get("gyro_offset_y");
    go.z = get("gyro_offset_z");
    mo.x = get("mag_offset_x");
    mo.y = get("mag_offset_y");
    mo.z = get("mag_offset_z");
    mo.r = get("mag_radius");
    return true;
  }

  // ── members ─────────────────────────────────────────────────────────────────

  int         i2c_bus_{1};
  u8          i2c_addr_{0x28};
  std::string calib_file_{};

  bno055_t sensor_{};
  bool     sensor_ready_{false};

  rclcpp::TimerBase::SharedPtr                    timer_;
  rclcpp::Publisher<DiagArray>::SharedPtr         diag_pub_;
  rclcpp::Service<Trigger>::SharedPtr             save_srv_;
  rclcpp::Service<Trigger>::SharedPtr             load_srv_;
};

}  // namespace bno055_hardware_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<bno055_hardware_interface::BNO055CalibNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("bno055_calib_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
