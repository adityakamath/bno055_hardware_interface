#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <limits>
#include <linux/i2c-dev.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

// smbus.h has no extern "C" guards — wrap manually so C++ links the C symbols
extern "C" {
#include <i2c/smbus.h>
}

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bno055_hardware_interface/bno055_hardware_interface.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// ── Hardware detection ────────────────────────────────────────────────────────

static constexpr int    kI2CBus  = 1;
static constexpr int    kI2CAddr = 0x28;
static constexpr uint8_t kBNO055ChipId = 0xA0;
static constexpr uint8_t kBNO055ChipIdReg = 0x00;

/// Returns true if BNO055 is present and responding on /dev/i2c-<kI2CBus>.
static bool bno055_available()
{
  std::string dev = "/dev/i2c-" + std::to_string(kI2CBus);
  int fd = ::open(dev.c_str(), O_RDWR);
  if (fd < 0) {return false;}
  if (::ioctl(fd, I2C_SLAVE, kI2CAddr) < 0) {::close(fd); return false;}
  auto chip_id = i2c_smbus_read_byte_data(fd, kBNO055ChipIdReg);
  ::close(fd);
  return static_cast<uint8_t>(chip_id) == kBNO055ChipId;
}

// Evaluated once at process start so all test bodies can cheaply query it.
static const bool kBNO055Available = bno055_available();

// ── HardwareInfo helpers ──────────────────────────────────────────────────────

/// Build a minimal valid HardwareInfo for the BNO055 IMU.
hardware_interface::HardwareInfo make_valid_imu_info(
  int i2c_bus            = kI2CBus,
  const std::string & i2c_addr  = "28",   // hex without 0x
  const std::string & axis_remap = "P1")
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters["i2c_bus"]    = std::to_string(i2c_bus);
  info.hardware_parameters["i2c_addr"]   = i2c_addr;
  info.hardware_parameters["axis_remap"] = axis_remap;

  hardware_interface::ComponentInfo sensor;
  sensor.name = "bno055";

  // Declare all 10 expected state interfaces (mirrors the URDF xacro)
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

// ── Lifecycle state helpers ───────────────────────────────────────────────────

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

// ════════════════════════════════════════════════════════════════════════════
// on_init: success cases (no hardware required)
// ════════════════════════════════════════════════════════════════════════════

TEST(InitTest, ValidDefaultConfig)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(InitTest, ValidAllAxisRemaps)
{
  for (const auto & remap : {"P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"}) {
    bno055_hardware_interface::BNO055HardwareInterface hw;
    auto info = make_valid_imu_info(kI2CBus, "28", remap);
    EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS)
      << "axis_remap=" << remap << " should be valid";
  }
}

TEST(InitTest, ValidAlternativeI2CAddress)
{
  // BNO055 can also be at 0x29 when ADR pin is high
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info(kI2CBus, "29");
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(InitTest, ValidCustomI2CBus)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info(0);  // bus 0
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(InitTest, EnableMockTrue)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.hardware_parameters["enable_mock"] = "true";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

TEST(InitTest, EnableMockOneString)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.hardware_parameters["enable_mock"] = "1";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::SUCCESS);
}

// ════════════════════════════════════════════════════════════════════════════
// on_init: error cases (no hardware required)
// ════════════════════════════════════════════════════════════════════════════

TEST(InitTest, NoSensorBlockFails)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.sensors.clear();  // remove the sensor
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(InitTest, TwoSensorBlocksFail)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.sensors.push_back(info.sensors[0]);  // duplicate
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(InitTest, InvalidAxisRemapFails)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info(kI2CBus, "28", "P9");  // P9 is not valid
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(InitTest, InvalidAxisRemapEmptyStringFails)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.hardware_parameters["axis_remap"] = "";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(InitTest, InvalidI2CBusNotANumberFails)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.hardware_parameters["i2c_bus"] = "abc";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

TEST(InitTest, InvalidI2CAddrNotHexFails)
{
  bno055_hardware_interface::BNO055HardwareInterface hw;
  auto info = make_valid_imu_info();
  info.hardware_parameters["i2c_addr"] = "XZ";
  EXPECT_EQ(hw.on_init(info), CallbackReturn::ERROR);
}

// ════════════════════════════════════════════════════════════════════════════
// export_state_interfaces (no hardware required)
// ════════════════════════════════════════════════════════════════════════════

class ExportStateInterfacesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<bno055_hardware_interface::BNO055HardwareInterface>();
    auto info = make_valid_imu_info();
    ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
    ifaces_ = hw_->export_state_interfaces();
  }

  std::unique_ptr<bno055_hardware_interface::BNO055HardwareInterface> hw_;
  std::vector<hardware_interface::StateInterface> ifaces_;
};

TEST_F(ExportStateInterfacesTest, ExactlyTenInterfaces)
{
  EXPECT_EQ(ifaces_.size(), 10u);
}

TEST_F(ExportStateInterfacesTest, AllExpectedNamesPresent)
{
  std::vector<std::string> names;
  for (const auto & iface : ifaces_) {
    names.push_back(iface.get_interface_name());
  }
  for (const auto & expected : {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"})
  {
    EXPECT_NE(std::find(names.begin(), names.end(), expected), names.end())
      << "Missing interface: " << expected;
  }
}

TEST_F(ExportStateInterfacesTest, InterfacesPrefixedWithSensorName)
{
  for (const auto & iface : ifaces_) {
    EXPECT_EQ(iface.get_prefix_name(), "bno055")
      << "Interface '" << iface.get_interface_name()
      << "' has unexpected prefix '" << iface.get_prefix_name() << "'";
  }
}

TEST_F(ExportStateInterfacesTest, InitialValuesAreIdentityQuaternion)
{
  // Before any read(), the interface values should be readable and match
  // the known defaults: identity quaternion (w=1), all others 0.
  for (auto & iface : ifaces_) {
    double val = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(iface.get_value(val, true))
      << "Could not read interface '" << iface.get_interface_name() << "'";

    double expected = 0.0;
    if (iface.get_interface_name() == "orientation.w") {
      expected = 1.0;  // identity quaternion — initialized to 1 in the hpp
    }
    EXPECT_DOUBLE_EQ(val, expected)
      << "Interface '" << iface.get_interface_name()
      << "' initial value is " << val << ", expected " << expected;
  }
}

// ════════════════════════════════════════════════════════════════════════════
// Lifecycle transitions — real hardware when available, mock mode otherwise
// ════════════════════════════════════════════════════════════════════════════

class HardwareLifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<bno055_hardware_interface::BNO055HardwareInterface>();
    auto info = make_valid_imu_info();
    if (!kBNO055Available) {
      // No hardware — use mock mode so the lifecycle tests still run
      info.hardware_parameters["enable_mock"] = "true";
    }
    ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    // Best-effort cleanup: put sensor in suspend and close I2C
    if (hw_) {
      hw_->on_cleanup(inactive_state());
    }
  }

  std::unique_ptr<bno055_hardware_interface::BNO055HardwareInterface> hw_;
};

TEST_F(HardwareLifecycleTest, ConfigureSucceeds)
{
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
}

TEST_F(HardwareLifecycleTest, ActivateAfterConfigureSucceeds)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_activate(inactive_state()), CallbackReturn::SUCCESS);
}

TEST_F(HardwareLifecycleTest, DeactivateAfterActivateSucceeds)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_activate(inactive_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_deactivate(active_state()), CallbackReturn::SUCCESS);
}

TEST_F(HardwareLifecycleTest, CleanupAfterConfigureSucceeds)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_cleanup(inactive_state()), CallbackReturn::SUCCESS);
}

TEST_F(HardwareLifecycleTest, ReconfigureAfterCleanupSucceeds)
{
  ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_cleanup(inactive_state()), CallbackReturn::SUCCESS);
  // Must create a fresh instance — re-init/configure the same object after cleanup
  hw_.reset();
  hw_ = std::make_unique<bno055_hardware_interface::BNO055HardwareInterface>();
  auto info2 = make_valid_imu_info();
  if (!kBNO055Available) {info2.hardware_parameters["enable_mock"] = "true";}
  ASSERT_EQ(hw_->on_init(info2), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
}

// ════════════════════════════════════════════════════════════════════════════
// read() behavior — real hardware when available, mock mode otherwise
// ════════════════════════════════════════════════════════════════════════════

class HardwareReadTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hw_ = std::make_unique<bno055_hardware_interface::BNO055HardwareInterface>();
    auto info = make_valid_imu_info();
    if (!kBNO055Available) {
      // No hardware — use mock mode so read() tests still run
      // Values stay frozen at defaults: orientation=(0,0,0,1), vel/accel=0
      info.hardware_parameters["enable_mock"] = "true";
    }
    ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);

    // Export interfaces BEFORE configure — they point into hw_'s internal storage
    ifaces_ = hw_->export_state_interfaces();

    ASSERT_EQ(hw_->on_configure(unconfigured_state()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_->on_activate(inactive_state()), CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    if (hw_) {
      hw_->on_deactivate(active_state());
      hw_->on_cleanup(inactive_state());
    }
  }

  /// Find a state interface by its interface name (e.g. "orientation.x").
  hardware_interface::StateInterface * find(const std::string & name)
  {
    for (auto & iface : ifaces_) {
      if (iface.get_interface_name() == name) {return &iface;}
    }
    return nullptr;
  }

  /// Get a double value from a named state interface.
  double get(const std::string & name)
  {
    auto * iface = find(name);
    if (!iface) {return std::numeric_limits<double>::quiet_NaN();}
    double v = std::numeric_limits<double>::quiet_NaN();
    iface->get_value(v, true);
    return v;
  }

  std::unique_ptr<bno055_hardware_interface::BNO055HardwareInterface> hw_;
  std::vector<hardware_interface::StateInterface> ifaces_;
  const rclcpp::Time   kTime{0, 0, RCL_ROS_TIME};
  const rclcpp::Duration kPeriod{0, static_cast<int32_t>(10e6)};  // 10 ms
};

TEST_F(HardwareReadTest, ReadReturnsOK)
{
  EXPECT_EQ(hw_->read(kTime, kPeriod), return_type::OK);
}

TEST_F(HardwareReadTest, QuaternionIsNearUnitNorm)
{
  // Allow a few reads for NDOF to settle
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(hw_->read(kTime, kPeriod), return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  const double qx = get("orientation.x");
  const double qy = get("orientation.y");
  const double qz = get("orientation.z");
  const double qw = get("orientation.w");

  ASSERT_FALSE(std::isnan(qx));
  ASSERT_FALSE(std::isnan(qy));
  ASSERT_FALSE(std::isnan(qz));
  ASSERT_FALSE(std::isnan(qw));

  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  EXPECT_NEAR(norm, 1.0, 0.05) << "Quaternion norm " << norm << " is not close to 1";
}

TEST_F(HardwareReadTest, AngularVelocityIsFinite)
{
  ASSERT_EQ(hw_->read(kTime, kPeriod), return_type::OK);

  EXPECT_TRUE(std::isfinite(get("angular_velocity.x")));
  EXPECT_TRUE(std::isfinite(get("angular_velocity.y")));
  EXPECT_TRUE(std::isfinite(get("angular_velocity.z")));
}

TEST_F(HardwareReadTest, LinearAccelerationIsFinite)
{
  ASSERT_EQ(hw_->read(kTime, kPeriod), return_type::OK);

  EXPECT_TRUE(std::isfinite(get("linear_acceleration.x")));
  EXPECT_TRUE(std::isfinite(get("linear_acceleration.y")));
  EXPECT_TRUE(std::isfinite(get("linear_acceleration.z")));
}

TEST_F(HardwareReadTest, MultipleReadsRemainStable)
{
  // Read repeatedly; API must keep returning OK (sensor errors are tolerated as WARN)
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(hw_->read(kTime, kPeriod), return_type::OK)
      << "read() failed at iteration " << i;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

TEST_F(HardwareReadTest, AllInterfacesUpdatedAfterRead)
{
  ASSERT_EQ(hw_->read(kTime, kPeriod), return_type::OK);

  for (auto & iface : ifaces_) {
    double v = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(iface.get_value(v, true))
      << "Could not read interface '" << iface.get_interface_name() << "'";
    EXPECT_FALSE(std::isnan(v))
      << "Interface '" << iface.get_interface_name() << "' returned NaN after read()";
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
