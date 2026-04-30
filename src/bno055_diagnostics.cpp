/** @file bno055_diagnostics.cpp
 *
 * Publishes BNO055 sensor health as a ROS 2 diagnostics message.
 *
 * Opens the I2C bus independently of the hardware interface plugin (the Linux
 * i2c-dev kernel driver serialises concurrent SMBus calls, so both can safely
 * share the bus). Reads status and calibration registers at 1 Hz and publishes a diagnostic_msgs/DiagnosticArray to /diagnostics.
 *
 * Compatible with rqt_robot_monitor, diagnostic_aggregator, and Foxglove's
 * diagnostics panel without any extra configuration.
 *
 * Parameters:
 *   i2c_bus     (int,    default 1)       I2C bus number → /dev/i2c-{n}
 *   i2c_addr    (string, default "28")    I2C address hex without 0x prefix
 *   sensor_mode (string, default "NDOF")  NDOF | NDOF_FMC_OFF | IMUPLUS
 *                                         IMUPLUS suppresses the MAG calibration entry
 */

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "bno055_hardware_interface/bno055_i2c.h"  // pulls in bno055.h inside extern "C"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

// ── helpers ───────────────────────────────────────────────────────────────────

static std::string sys_status_string(uint8_t code)
{
  switch (code) {
    case 0: return "Idle";
    case 1: return "System Error";
    case 2: return "Initializing Peripherals";
    case 3: return "System Initialization";
    case 4: return "Executing Self-Test";
    case 5: return "Sensor Fusion Running";
    case 6: return "Running Without Fusion";
    default: return "Unknown (" + std::to_string(code) + ")";
  }
}

static std::string sys_error_string(uint8_t code)
{
  switch (code) {
    case 0: return "No Error";
    case 1: return "Peripheral Initialization Error";
    case 2: return "System Initialization Error";
    case 3: return "Self-Test Result Failed";
    case 4: return "Register Map Value Out of Range";
    case 5: return "Register Map Address Out of Range";
    case 6: return "Register Map Write Error";
    case 7: return "Low Power Mode Not Available";
    case 8: return "Accelerometer Power Mode Not Available";
    case 9: return "Fusion Algorithm Configuration Error";
    default: return "Unknown (" + std::to_string(code) + ")";
  }
}

static KeyValue kv(const std::string & key, const std::string & value)
{
  return KeyValue{}.set__key(key).set__value(value);
}

// Helper to parse a bool parameter from rclcpp::Node, accepting both bool and string types
static bool parse_bool_param(rclcpp::Node* node, const std::string& name, bool default_value) {
  if (!node->has_parameter(name)) {
    return default_value;
  }
  const auto& param = node->get_parameter(name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return param.as_bool();
  } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    std::string val = param.as_string();
    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    return (val == "true" || val == "1" || val == "yes" || val == "on");
  }
  return default_value;
}

// ── node ──────────────────────────────────────────────────────────────────────

class BNO055DiagnosticsNode : public rclcpp::Node
{

public:
  explicit BNO055DiagnosticsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("bno055_diagnostics", options)
  {
    declare_parameter("i2c_bus",     1);
    declare_parameter("i2c_addr",    std::string("28"));
    declare_parameter("sensor_mode", std::string("NDOF"));
    declare_parameter("enable_mock_mode", std::string("false"));

    i2c_bus_     = get_parameter("i2c_bus").as_int();
    sensor_mode_ = get_parameter("sensor_mode").as_string();
    enable_mock_ = parse_bool_param(this, "enable_mock_mode", false);

    // Parse i2c_addr hex string
    try {
      i2c_addr_ = static_cast<uint8_t>(
        std::stoul(get_parameter("i2c_addr").as_string(), nullptr, 16));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Invalid i2c_addr parameter: %s", e.what());
      throw;
    }

    // Precompute hardware_id string — used in every diagnostic publish
    if (enable_mock_) {
      hardware_id_ = "mock";
    } else {
      std::ostringstream oss;
      oss << "/dev/i2c-" << i2c_bus_ << " @ 0x"
          << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
          << static_cast<int>(i2c_addr_);
      hardware_id_ = oss.str();
    }

    if (enable_mock_) {
      RCLCPP_INFO(get_logger(), "BNO055 diagnostics running in mock mode");
    } else {
      // Open the I2C bus and initialise the Bosch driver
      std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);
      if (bno055_i2c_open(device.c_str(), i2c_addr_) != 0) {
        RCLCPP_FATAL(
          get_logger(), "Failed to open %s at 0x%02X", device.c_str(), i2c_addr_);
        throw std::runtime_error("I2C open failed");
      }

      sensor_.bus_read   = BNO055_I2C_bus_read;
      sensor_.bus_write  = BNO055_I2C_bus_write;
      sensor_.delay_msec = BNO055_delay_msek;
      sensor_.dev_addr   = i2c_addr_;

      if (bno055_init(&sensor_) != BNO055_SUCCESS) {
        bno055_i2c_close();
        RCLCPP_FATAL(get_logger(), "bno055_init() failed — check wiring and I2C address");
        throw std::runtime_error("bno055_init failed");
      }

      RCLCPP_INFO(
        get_logger(),
        "BNO055 diagnostics started: %s 0x%02X sensor_mode=%s",
        device.c_str(), i2c_addr_, sensor_mode_.c_str());
    }

    pub_ = create_publisher<DiagnosticArray>("/diagnostics", rclcpp::SystemDefaultsQoS());
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1s, [this]() { publish(); });
  }

  ~BNO055DiagnosticsNode()
  {
    if (!enable_mock_) {
      bno055_i2c_close();
    }
  }

private:
  void publish()
  {
    if (enable_mock_) {
      // Publish realistic simulated values — same key-value structure as real hardware,
      // representing a healthy, fully-calibrated sensor at ambient temperature.
      DiagnosticStatus status;
      status.name        = "BNO055 IMU";
      status.hardware_id = hardware_id_;
      status.level       = DiagnosticStatus::OK;
      status.message     = "Sensor Fusion Running (mock)";
      status.values.push_back(kv("System Status",     "Sensor Fusion Running"));
      status.values.push_back(kv("Calibration (SYS)", "3/3"));
      status.values.push_back(kv("Calibration (GYR)", "3/3"));
      status.values.push_back(kv("Calibration (ACC)", "3/3"));
      if (sensor_mode_ != "IMUPLUS") {
        status.values.push_back(kv("Calibration (MAG)", "3/3"));
      }
      status.values.push_back(kv("Temperature",       "25.0 °C"));
      DiagnosticArray msg;
      msg.header.stamp = now();
      msg.status.push_back(std::move(status));
      pub_->publish(msg);
      return;
    }

    DiagnosticStatus status;
    status.name        = "BNO055 IMU";
    status.hardware_id = hardware_id_;

    uint8_t sys_status = 0, sys_error = 0;
    uint8_t calib_sys = 0, calib_gyro = 0, calib_accel = 0, calib_mag = 0;
    double temp_c = 0.0;

    s32 rc = BNO055_SUCCESS;
    rc += bno055_get_sys_stat_code(&sys_status);
    rc += bno055_get_sys_error_code(&sys_error);
    rc += bno055_get_sys_calib_stat(&calib_sys);
    rc += bno055_get_gyro_calib_stat(&calib_gyro);
    rc += bno055_get_accel_calib_stat(&calib_accel);
    rc += bno055_get_mag_calib_stat(&calib_mag);
    rc += bno055_convert_double_temp_celsius(&temp_c);

    if (rc != BNO055_SUCCESS) {
      status.level   = DiagnosticStatus::ERROR;
      status.message = "I2C read error — sensor disconnected?";
      status.values.push_back(kv("Return Code", std::to_string(rc)));
    } else if (sys_status == 1) {
      // System error state
      status.level   = DiagnosticStatus::ERROR;
      status.message = "System Error: " + sys_error_string(sys_error);
      status.values.push_back(kv("System Status", sys_status_string(sys_status)));
      status.values.push_back(kv("System Error",  sys_error_string(sys_error)));
    } else {
      // Determine calibration health
      bool mag_relevant = (sensor_mode_ != "IMUPLUS");
      bool fully_calibrated =
        calib_sys >= 1 && calib_gyro >= 1 && calib_accel >= 1 &&
        (!mag_relevant || calib_mag >= 1);

      if (!fully_calibrated) {
        status.level   = DiagnosticStatus::WARN;
        status.message = "Calibrating — move sensor to improve accuracy";
      } else {
        status.level   = DiagnosticStatus::OK;
        status.message = sys_status_string(sys_status);
      }

      status.values.push_back(kv("System Status",    sys_status_string(sys_status)));
      status.values.push_back(kv("Calibration (SYS)", std::to_string(calib_sys)  + "/3"));
      status.values.push_back(kv("Calibration (GYR)", std::to_string(calib_gyro) + "/3"));
      status.values.push_back(kv("Calibration (ACC)", std::to_string(calib_accel) + "/3"));
      if (mag_relevant) {
        status.values.push_back(kv("Calibration (MAG)", std::to_string(calib_mag) + "/3"));
      }
      // Temperature from gyroscope die — useful context when calibration drifts
      char temp_buf[16];
      snprintf(temp_buf, sizeof(temp_buf), "%.1f °C", temp_c);
      status.values.push_back(kv("Temperature", temp_buf));
    }

    DiagnosticArray msg;
    msg.header.stamp = now();
    msg.status.push_back(std::move(status));
    pub_->publish(msg);
  }

  int         i2c_bus_{1};
  uint8_t     i2c_addr_{0x28};
  std::string sensor_mode_{"NDOF"};
  bool        enable_mock_{false};
  std::string hardware_id_{};     // precomputed in constructor

  bno055_t  sensor_{};

  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BNO055DiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}
