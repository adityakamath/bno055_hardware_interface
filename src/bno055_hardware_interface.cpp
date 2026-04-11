/** @file bno055_hardware_interface.cpp
 *
 * ros2_control SensorInterface for the Bosch BNO055 IMU over I2C.
 *
 * Uses the official Bosch Sensortec BNO055 C driver (external/BNO055_driver)
 * wired to the Linux i2c-dev / SMBus kernel interface via bno055_i2c.c.
 * The I2C adapter approach follows bdholt1/ros2_bno055_sensor (Apache-2.0).
 * Register layout and scaling constants follow flynneva/bno055 (BSD-3).
 */

#include "bno055_hardware_interface/bno055_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace
{

bool parse_calib_yaml(
  const std::string & path,
  bno055_accel_offset_t & ao,
  bno055_gyro_offset_t & go,
  bno055_mag_offset_t & mo)
{
  std::ifstream f(path);
  if (!f) {return false;}

  std::unordered_map<std::string, int16_t> vals;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    const auto colon = line.find(':');
    if (colon == std::string::npos) {continue;}
    std::string key = line.substr(0, colon);
    const auto key_end = key.find_last_not_of(" \t\r");
    key = (key_end == std::string::npos) ? "" : key.substr(0, key_end + 1);
    try {
      vals[key] = static_cast<int16_t>(std::stoi(line.substr(colon + 1)));
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("bno055_calib"),
        "Ignoring malformed calibration value for '%s': %s", key.c_str(), e.what());
      continue;
    }
  }

  auto get = [&](const std::string & k) -> int16_t {
      const auto it = vals.find(k);
      return (it != vals.end()) ? it->second : int16_t(0);
    };

  ao = bno055_accel_offset_t{};
  ao.x = get("accel_offset_x");
  ao.y = get("accel_offset_y");
  ao.z = get("accel_offset_z");
  ao.r = get("accel_radius");

  go = bno055_gyro_offset_t{};
  go.x = get("gyro_offset_x");
  go.y = get("gyro_offset_y");
  go.z = get("gyro_offset_z");

  mo = bno055_mag_offset_t{};
  mo.x = get("mag_offset_x");
  mo.y = get("mag_offset_y");
  mo.z = get("mag_offset_z");
  mo.r = get("mag_radius");

  return true;
}

// Quaternion raw -> unit: divide by 2^14 (Bosch datasheet §3.6.5.5)
constexpr double QUATERNION_SCALE = 1.0 / 16384.0;

// Axis remap lookup: P0-P7 -> { AXIS_MAP_CONFIG byte, AXIS_MAP_SIGN byte }
// Values from BNO055 datasheet Table 3-4 (same as flynneva/bno055)
const std::map<std::string, std::pair<uint8_t, uint8_t>> kAxisRemap = {
  {"P0", {0x21, 0x04}},
  {"P1", {0x24, 0x00}},
  {"P2", {0x24, 0x06}},
  {"P3", {0x21, 0x02}},
  {"P4", {0x24, 0x03}},
  {"P5", {0x21, 0x01}},
  {"P6", {0x21, 0x07}},
  {"P7", {0x24, 0x05}},
};

// Fusion mode lookup: parameter string -> BNO055 operation mode constant.
// All three modes produce identical outputs: quaternion + angular_velocity + linear_acceleration.
//   NDOF         - 9-DOF, absolute orientation anchored to magnetic North
//   NDOF_FMC_OFF - same as NDOF but fast magnetometer calibration disabled (for noisy environments)
//   IMUPLUS      - 6-DOF, relative orientation, gyro + accel only (no magnetometer)
const std::map<std::string, uint8_t> kOperationMode = {
  {"NDOF",         BNO055_OPERATION_MODE_NDOF},
  {"NDOF_FMC_OFF", BNO055_OPERATION_MODE_NDOF_FMC_OFF},
  {"IMUPLUS",      BNO055_OPERATION_MODE_IMUPLUS},
};

}  // namespace

namespace bno055_hardware_interface
{

// ── on_init: parse URDF hardware parameters ──────────────────────────────────

hardware_interface::CallbackReturn BNO055HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SensorInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
#pragma GCC diagnostic pop
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Initializing BNO055 hardware interface: %s", info_.name.c_str());

  // i2c_bus (default: 1)
  if (const auto it = info_.hardware_parameters.find("i2c_bus");
    it != info_.hardware_parameters.end())
  {
    try {
      i2c_bus_ = std::stoi(it->second);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Invalid i2c_bus: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // i2c_addr (default: 0x28)
  if (const auto it = info_.hardware_parameters.find("i2c_addr");
    it != info_.hardware_parameters.end())
  {
    try {
      i2c_addr_ = static_cast<uint8_t>(std::stoul(it->second, nullptr, 16));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Invalid i2c_addr: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // axis_remap (default: "P1")
  if (const auto it = info_.hardware_parameters.find("axis_remap");
    it != info_.hardware_parameters.end())
  {
    axis_remap_ = it->second;
  }
  if (kAxisRemap.find(axis_remap_) == kAxisRemap.end()) {
    RCLCPP_ERROR(logger_, "Invalid axis_remap '%s'. Must be P0-P7.", axis_remap_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // enable_mock_mode (default: false)
  enable_mock_ = parse_bool_param("enable_mock_mode", false);

  // calib_file (default: empty — no file, rely on in-sensor calibration)
  if (const auto it = info_.hardware_parameters.find("calib_file");
    it != info_.hardware_parameters.end())
  {
    calib_file_ = it->second;
  }

  // sensor_mode (default: "NDOF")
  if (const auto it = info_.hardware_parameters.find("sensor_mode");
    it != info_.hardware_parameters.end())
  {
    sensor_mode_ = it->second;
  }
  if (kOperationMode.find(sensor_mode_) == kOperationMode.end()) {
    RCLCPP_ERROR(
      logger_, "Invalid sensor_mode '%s'. Must be one of: NDOF, NDOF_FMC_OFF, IMUPLUS.",
      sensor_mode_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(logger_, "Expected exactly 1 <sensor> element, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate that each declared state interface name matches one of the 10 expected.
  const std::vector<std::string> kExpected = {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
  };
  for (const auto & si : info_.sensors[0].state_interfaces) {
    if (std::find(kExpected.begin(), kExpected.end(), si.name) == kExpected.end()) {
      RCLCPP_ERROR(
        logger_, "Unexpected state interface '%s'. Expected one of: "
        "orientation.{x,y,z,w}, angular_velocity.{x,y,z}, linear_acceleration.{x,y,z}",
        si.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    logger_,
    "Initialized: i2c_bus=%d i2c_addr=0x%02X axis_remap=%s sensor_mode=%s mock=%s calib_file=%s",
    i2c_bus_, i2c_addr_, axis_remap_.c_str(), sensor_mode_.c_str(),
    enable_mock_ ? "true" : "false",
    calib_file_.empty() ? "(none)" : calib_file_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure: open I2C, init Bosch driver, configure BNO055 ──────────────

hardware_interface::CallbackReturn BNO055HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring BNO055...");
  consecutive_read_errors_ = 0;

  if (enable_mock_) {
    RCLCPP_INFO(logger_, "Mock mode enabled - skipping I2C initialization");
    // Identity quaternion: represents no rotation (robot aligned with world frame).
    // A zero quaternion is mathematically invalid and would produce NaN in the EKF.
    hw_orientation_w_ = 1.0;
    hw_orientation_x_ = 0.0;
    hw_orientation_y_ = 0.0;
    hw_orientation_z_ = 0.0;
    // Angular velocity and linear acceleration stay at 0.0 (correct for a stationary mock).
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Open I2C bus
  std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);
  if (bno055_i2c_open(device.c_str(), i2c_addr_) != 0) {
    RCLCPP_ERROR(logger_, "Failed to open I2C device %s at address 0x%02X",
      device.c_str(), i2c_addr_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Wire Bosch driver callbacks
  sensor_.bus_read  = BNO055_I2C_bus_read;
  sensor_.bus_write = BNO055_I2C_bus_write;
  sensor_.delay_msec = BNO055_delay_msek;
  sensor_.dev_addr  = static_cast<u8>(i2c_addr_);

  // Initialize the Bosch driver (reads chip ID, populates rev info)
  s32 comres = bno055_init(&sensor_);
  if (comres != BNO055_SUCCESS) {
    RCLCPP_ERROR(logger_, "bno055_init() failed (err=%d). Check wiring and I2C address.", comres);
    bno055_i2c_close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "BNO055 detected: chip_id=0x%02X sw_rev=0x%04X",
    sensor_.chip_id, sensor_.sw_rev_id);

  // Switch to CONFIG mode to apply settings
  comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
  if (comres != BNO055_SUCCESS) {
    RCLCPP_WARN(logger_, "Failed to set CONFIG mode (err=%d)", comres);
  }
  rclcpp::sleep_for(std::chrono::milliseconds(25));  // datasheet: >=19 ms

  // Normal power mode
  comres = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
  if (comres != BNO055_SUCCESS) {
    RCLCPP_WARN(logger_, "Failed to set NORMAL power mode (err=%d)", comres);
  }

  // Gyro unit: radians per second
  comres = bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);
  if (comres != BNO055_SUCCESS) {
    RCLCPP_ERROR(logger_, "Failed to set gyro unit to rps (err=%d)", comres);
    bno055_i2c_close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Accel unit: m/s^2
  comres = bno055_set_accel_unit(BNO055_ACCEL_UNIT_MSQ);
  if (comres != BNO055_SUCCESS) {
    RCLCPP_ERROR(logger_, "Failed to set accel unit to m/s^2 (err=%d)", comres);
    bno055_i2c_close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Apply saved calibration offsets if a file was provided.
  // Offsets must be written in CONFIG mode, before switching to the configured fusion mode.
  if (!calib_file_.empty()) {
    if (load_calib_offsets()) {
      RCLCPP_INFO(logger_, "Calibration offsets loaded from %s", calib_file_.c_str());
    } else {
      RCLCPP_WARN(
        logger_,
        "Calibration file '%s' not found or unreadable — starting uncalibrated.",
        calib_file_.c_str());
    }
  }

  // Axis remap: write { AXIS_MAP_CONFIG, AXIS_MAP_SIGN } directly via the
  // wired bus_write callback (same values as flynneva/bno055 P-code table)
  const auto & remap = kAxisRemap.at(axis_remap_);
  u8 remap_cfg  = remap.first;
  u8 remap_sign = remap.second;
  if (sensor_.bus_write(sensor_.dev_addr, BNO055_AXIS_MAP_CONFIG_ADDR, &remap_cfg, 1) != 0) {
    RCLCPP_WARN(logger_, "Axis remap: failed to write AXIS_MAP_CONFIG register");
  }
  if (sensor_.bus_write(sensor_.dev_addr, BNO055_AXIS_MAP_SIGN_ADDR, &remap_sign, 1) != 0) {
    RCLCPP_WARN(logger_, "Axis remap: failed to write AXIS_MAP_SIGN register");
  }
  RCLCPP_INFO(logger_, "Axis remap %s applied (cfg=0x%02X sign=0x%02X)",
    axis_remap_.c_str(), remap_cfg, remap_sign);

  // Switch to configured fusion mode
  comres = bno055_set_operation_mode(kOperationMode.at(sensor_mode_));
  if (comres != BNO055_SUCCESS) {
    RCLCPP_ERROR(
      logger_, "Failed to set %s operation mode (err=%d)", sensor_mode_.c_str(), comres);
    bno055_i2c_close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  rclcpp::sleep_for(std::chrono::milliseconds(20));  // datasheet: >=7 ms

  RCLCPP_INFO(logger_, "BNO055 configured in %s fusion mode", sensor_mode_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── load_calib_offsets: read YAML file and write offsets to sensor ───────────
// Called in on_configure while the sensor is still in CONFIG mode.

bool BNO055HardwareInterface::load_calib_offsets()
{
  bno055_accel_offset_t ao{};
  bno055_gyro_offset_t go{};
  bno055_mag_offset_t mo{};

  if (!parse_calib_yaml(calib_file_, ao, go, mo)) {return false;}

  bool ok =
    (bno055_write_accel_offset(&ao) == BNO055_SUCCESS) &&
    (bno055_write_gyro_offset(&go)  == BNO055_SUCCESS) &&
    (bno055_write_mag_offset(&mo)   == BNO055_SUCCESS);

  if (ok) {
    RCLCPP_INFO(
      logger_,
      "  Accel offsets: x=%d y=%d z=%d  radius=%d", ao.x, ao.y, ao.z, ao.r);
    RCLCPP_INFO(
      logger_,
      "  Gyro offsets:  x=%d y=%d z=%d", go.x, go.y, go.z);
    RCLCPP_INFO(
      logger_,
      "  Mag offsets:   x=%d y=%d z=%d  radius=%d", mo.x, mo.y, mo.z, mo.r);
  } else {
    RCLCPP_ERROR(logger_, "Failed to write calibration offsets to sensor");
  }
  return ok;
}

// ── on_activate / on_deactivate / on_cleanup ─────────────────────────────────

hardware_interface::CallbackReturn BNO055HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "BNO055 hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "BNO055 hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_hardware();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_hardware();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055HardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "BNO055 hardware interface entering error recovery — closing hardware");
  close_hardware();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void BNO055HardwareInterface::close_hardware()
{
  if (!enable_mock_) {
    bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
    bno055_i2c_close();
    RCLCPP_INFO(logger_, "BNO055 suspended and I2C closed");
  }
  hw_orientation_x_ = 0.0;
  hw_orientation_y_ = 0.0;
  hw_orientation_z_ = 0.0;
  hw_orientation_w_ = 1.0;
  hw_angular_velocity_x_    = 0.0;
  hw_angular_velocity_y_    = 0.0;
  hw_angular_velocity_z_    = 0.0;
  hw_linear_acceleration_x_ = 0.0;
  hw_linear_acceleration_y_ = 0.0;
  hw_linear_acceleration_z_ = 0.0;
}

// ── export_state_interfaces ───────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
BNO055HardwareInterface::export_state_interfaces()
{
  const std::string & sensor_name = info_.sensors[0].name;
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(sensor_name, "orientation.x",         &hw_orientation_x_);
  state_interfaces.emplace_back(sensor_name, "orientation.y",         &hw_orientation_y_);
  state_interfaces.emplace_back(sensor_name, "orientation.z",         &hw_orientation_z_);
  state_interfaces.emplace_back(sensor_name, "orientation.w",         &hw_orientation_w_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.x",    &hw_angular_velocity_x_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.y",    &hw_angular_velocity_y_);
  state_interfaces.emplace_back(sensor_name, "angular_velocity.z",    &hw_angular_velocity_z_);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.x", &hw_linear_acceleration_x_);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.y", &hw_linear_acceleration_y_);
  state_interfaces.emplace_back(sensor_name, "linear_acceleration.z", &hw_linear_acceleration_z_);

  RCLCPP_INFO(logger_, "Exported 10 state interfaces for sensor '%s'", sensor_name.c_str());
  return state_interfaces;
}

// ── read: poll BNO055 and update state interfaces ────────────────────────────

hardware_interface::return_type BNO055HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (enable_mock_) {
    return hardware_interface::return_type::OK;
  }

  // Orientation — raw quaternion (s16 values, scale = 1/2^14)
  bno055_quaternion_t quat;
  // Angular velocity — raw s16 counts; unit was set to RPS in on_configure
  bno055_gyro_t gyro;
  // Linear acceleration — raw s16 counts; unit was set to m/s² in on_configure
  bno055_linear_accel_t lin_accel;

  // Use raw read functions + manual scaling to avoid the convert_double variants,
  // which re-read the unit register on every call and may trigger a CONFIG mode
  // switch if the register is ever corrupted (stalling fusion for 19+ ms).
  bool ok =
    (bno055_read_quaternion_wxyz(&quat) == BNO055_SUCCESS) &&
    (bno055_read_gyro_xyz(&gyro) == BNO055_SUCCESS) &&
    (bno055_read_linear_accel_xyz(&lin_accel) == BNO055_SUCCESS);

  if (!ok) {
    ++consecutive_read_errors_;
    if (consecutive_read_errors_ >= 10) {
      RCLCPP_ERROR(
        logger_, "BNO055: %d consecutive read failures — reporting ERROR",
        consecutive_read_errors_);
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN(logger_, "BNO055 read error (streak=%d) — keeping previous values",
      consecutive_read_errors_);
    return hardware_interface::return_type::OK;
  }
  consecutive_read_errors_ = 0;

  // Normalize quaternion (raw values scale to unit quaternion via QUATERNION_SCALE)
  const double qw = quat.w * QUATERNION_SCALE;
  const double qx = quat.x * QUATERNION_SCALE;
  const double qy = quat.y * QUATERNION_SCALE;
  const double qz = quat.z * QUATERNION_SCALE;
  const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (norm > 1e-6) {
    hw_orientation_w_ = qw / norm;
    hw_orientation_x_ = qx / norm;
    hw_orientation_y_ = qy / norm;
    hw_orientation_z_ = qz / norm;
  }

  hw_angular_velocity_x_ = gyro.x / BNO055_GYRO_DIV_RPS;
  hw_angular_velocity_y_ = gyro.y / BNO055_GYRO_DIV_RPS;
  hw_angular_velocity_z_ = gyro.z / BNO055_GYRO_DIV_RPS;

  hw_linear_acceleration_x_ = lin_accel.x / BNO055_LINEAR_ACCEL_DIV_MSQ;
  hw_linear_acceleration_y_ = lin_accel.y / BNO055_LINEAR_ACCEL_DIV_MSQ;
  hw_linear_acceleration_z_ = lin_accel.z / BNO055_LINEAR_ACCEL_DIV_MSQ;

  return hardware_interface::return_type::OK;
}

bool BNO055HardwareInterface::parse_bool_param(
  const std::string & key, bool default_value) const
{
  auto it = info_.hardware_parameters.find(key);
  if (it == info_.hardware_parameters.end()) {return default_value;}
  return it->second == "true";
}

}  // namespace bno055_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  bno055_hardware_interface::BNO055HardwareInterface,
  hardware_interface::SensorInterface)
