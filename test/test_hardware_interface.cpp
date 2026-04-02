#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bno055_hardware_interface/bno055_hardware_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// ── helpers ───────────────────────────────────────────────────────────────────

hardware_interface::HardwareInfo make_valid_imu_info(
  int i2c_bus = 1,
  const std::string & i2c_addr = "28",
  const std::string & axis_remap = "P1")
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["i2c_bus"]    = std::to_string(i2c_bus);
  info.hardware_parameters["i2c_addr"]   = i2c_addr;
  info.hardware_parameters["axis_remap"] = axis_remap;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bno055";
  for (const auto & name : {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"})
  {
    hardware_interface::InterfaceInfo iface;
    iface.name = name;
    sensor.state_interfaces.push_back(iface);
  }
  info.sensors.push_back(sensor);
  return info;
}

static rclcpp_lifecycle::State unconfigured_state()
{
  return rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
}
static rclcpp_lifecycle::State inactive_state()
{
  return rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}
static rclcpp_lifecycle::State active_state()
{
  return rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
}

// ── on_init ───────────────────────────────────────────────────────────────────

TEST(InitTest, ValidParams)
{
  auto init = [](hardware_interface::HardwareInfo info) {
      bno055_hardware_interface::BNO055HardwareInterface hw;
      return hw.on_init(info);
    };

  EXPECT_EQ(init(make_valid_imu_info()), CallbackReturn::SUCCESS);
  EXPECT_EQ(init(make_valid_imu_info(0, "29")), CallbackReturn::SUCCESS);  // bus 0, addr 0x29

  auto mock = make_valid_imu_info();
  mock.hardware_parameters["enable_mock"] = "true";
  EXPECT_EQ(init(mock), CallbackReturn::SUCCESS);

  auto calib = make_valid_imu_info();
  calib.hardware_parameters["calib_file"] = "~/.ros/bno055_calib.yaml";
  EXPECT_EQ(init(calib), CallbackReturn::SUCCESS);
}

TEST(InitTest, ValidAllSensorModes)
{
  for (const auto & mode : {"NDOF", "NDOF_FMC_OFF", "IMUPLUS"}) {
    bno055_hardware_interface::BNO055HardwareInterface hw;
    auto info = make_valid_imu_info();
    info.hardware_parameters["sensor_mode"] = mode;
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS) << "sensor_mode=" << mode;
  }
}

TEST(InitTest, ValidAllAxisRemaps)
{
  for (const auto & remap : {"P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"}) {
    bno055_hardware_interface::BNO055HardwareInterface hw;
    EXPECT_EQ(hw.on_init(make_valid_imu_info(1, "28", remap)), CallbackReturn::SUCCESS)
      << "axis_remap=" << remap;
  }
}

TEST(InitTest, InvalidParamsFail)
{
  auto init = [](hardware_interface::HardwareInfo info) {
      bno055_hardware_interface::BNO055HardwareInterface hw;
      return hw.on_init(info);
    };

  auto no_sensor = make_valid_imu_info();
  no_sensor.sensors.clear();
  EXPECT_EQ(init(no_sensor), CallbackReturn::ERROR);

  auto two_sensors = make_valid_imu_info();
  two_sensors.sensors.push_back(two_sensors.sensors[0]);
  EXPECT_EQ(init(two_sensors), CallbackReturn::ERROR);

  EXPECT_EQ(init(make_valid_imu_info(1, "28", "P9")), CallbackReturn::ERROR);

  auto empty_remap = make_valid_imu_info();
  empty_remap.hardware_parameters["axis_remap"] = "";
  EXPECT_EQ(init(empty_remap), CallbackReturn::ERROR);

  auto bad_bus = make_valid_imu_info();
  bad_bus.hardware_parameters["i2c_bus"] = "abc";
  EXPECT_EQ(init(bad_bus), CallbackReturn::ERROR);

  auto bad_addr = make_valid_imu_info();
  bad_addr.hardware_parameters["i2c_addr"] = "XZ";
  EXPECT_EQ(init(bad_addr), CallbackReturn::ERROR);

  auto bad_mode = make_valid_imu_info();
  bad_mode.hardware_parameters["sensor_mode"] = "ACCGYRO";
  EXPECT_EQ(init(bad_mode), CallbackReturn::ERROR);
}

// ── export_state_interfaces ───────────────────────────────────────────────────

TEST(ExportStateInterfacesTest, InterfacesCorrect)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  ASSERT_EQ(hw.on_init(make_valid_imu_info()), CallbackReturn::SUCCESS);
  auto ifaces = hw.export_state_interfaces();

  ASSERT_EQ(ifaces.size(), 10u);

  std::vector<std::string> names;
  for (const auto & iface : ifaces) {
    EXPECT_EQ(iface.get_prefix_name(), "bno055");
    names.push_back(iface.get_interface_name());
  }
  for (const auto & expected : {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"})
  {
    EXPECT_NE(std::find(names.begin(), names.end(), expected), names.end())
      << "Missing: " << expected;
  }

  // Initial values: identity quaternion, all others 0
  for (auto & iface : ifaces) {
    double val = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(iface.get_value(val, true));
    const double expected = (iface.get_interface_name() == "orientation.w") ? 1.0 : 0.0;
    EXPECT_DOUBLE_EQ(val, expected) << iface.get_interface_name();
  }
}

// ── lifecycle + read (mock mode) ──────────────────────────────────────────────

class MockHwTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<bno055_hardware_interface::BNO055HardwareInterface>();
    auto info = make_valid_imu_info();
    info.hardware_parameters["enable_mock"] = "true";
    ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
  }

  double get(const std::string & name)
  {
    for (auto & iface : ifaces_) {
      if (iface.get_interface_name() == name) {
        double v = std::numeric_limits<double>::quiet_NaN();
        (void)iface.get_value(v, true);
        return v;
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::unique_ptr<bno055_hardware_interface::BNO055HardwareInterface> hw_;
  std::vector<hardware_interface::StateInterface> ifaces_;
  const rclcpp::Time     kTime{0, 0, RCL_ROS_TIME};
  const rclcpp::Duration kPeriod{0, static_cast<int32_t>(10e6)};
};

TEST_F(MockHwTest, FullLifecycle)
{
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_deactivate(active_state()),      CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_cleanup(inactive_state()),       CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, ReconfigureAfterCleanup)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_cleanup(inactive_state()),       CallbackReturn::SUCCESS);

  hw_ = std::make_unique<bno055_hardware_interface::BNO055HardwareInterface>();
  auto info = make_valid_imu_info();
  info.hardware_parameters["enable_mock"] = "true";
  ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
}

TEST_F(MockHwTest, ReadOutputsValid)
{
  ifaces_ = hw_->export_state_interfaces();
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->read(kTime, kPeriod), return_type::OK);

  // Mock: identity quaternion, zeros elsewhere — all finite and non-NaN
  EXPECT_DOUBLE_EQ(get("orientation.w"), 1.0);
  EXPECT_DOUBLE_EQ(get("orientation.x"), 0.0);
  EXPECT_DOUBLE_EQ(get("orientation.y"), 0.0);
  EXPECT_DOUBLE_EQ(get("orientation.z"), 0.0);
  for (auto & iface : ifaces_) {
    double v = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(iface.get_value(v, true));
    EXPECT_FALSE(std::isnan(v)) << iface.get_interface_name();
    EXPECT_TRUE(std::isfinite(v)) << iface.get_interface_name();
  }

  // Quaternion unit norm
  const double qw = get("orientation.w"), qx = get("orientation.x"),
    qy = get("orientation.y"), qz = get("orientation.z");
  EXPECT_NEAR(std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz), 1.0, 0.05);
}

TEST_F(MockHwTest, MultipleReadsRemainStable)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()),      CallbackReturn::SUCCESS);
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(hw_->read(kTime, kPeriod), return_type::OK) << "iteration " << i;
  }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
