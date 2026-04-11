cmake_minimum_required(VERSION 3.28)
project(bno055_hardware_interface VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Bosch Sensortec BNO055 C driver (external submodule) ────────────────────
set(BNO055_DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/BNO055_driver)

add_library(bno055_driver STATIC
  ${BNO055_DRIVER_DIR}/bno055.c
)
target_include_directories(bno055_driver PUBLIC
  ${BNO055_DRIVER_DIR}
)
# Bosch driver uses a large switch statement; suppress pedantic warning
target_compile_options(bno055_driver PRIVATE -Wno-pedantic -Wno-unused-variable)

# ── I2C adapter (C, wraps libi2c smbus calls for the Bosch driver) ───────────
add_library(bno055_i2c STATIC
  src/bno055_i2c.c
)
target_include_directories(bno055_i2c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${BNO055_DRIVER_DIR}
)
target_link_libraries(bno055_i2c i2c bno055_driver)


# ── BNO055 Hardware Interface (ros2_control sensor plugin) ──────────────────
add_library(${PROJECT_NAME} SHARED
  src/bno055_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${BNO055_DRIVER_DIR}
)

target_link_libraries(${PROJECT_NAME}
  bno055_driver
  bno055_i2c
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware interface as plugin
pluginlib_export_plugin_description_file(hardware_interface bno055_hardware_interface.xml)

# ── IMU TF broadcaster (optional visualization helper) ───────────────────────
add_executable(imu_tf_broadcaster
  src/imu_tf_broadcaster.cpp
)
target_link_libraries(imu_tf_broadcaster
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
)
install(TARGETS imu_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# ── BNO055 Diagnostics node (optional companion, publishes /diagnostics at 1 Hz) ─
add_executable(bno055_diagnostics
  src/bno055_diagnostics.cpp
)
target_include_directories(bno055_diagnostics PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${BNO055_DRIVER_DIR}
)
target_link_libraries(bno055_diagnostics
  bno055_driver
  bno055_i2c
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
)
install(TARGETS bno055_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES bno055_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Suppress only the C/C++ style linters that recurse into the external/ Bosch
  # vendor tree and trip on its non-standard formatting conventions.
  # copyright and lint_cmake do not inspect external C sources, so they run normally.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # C++ unit / integration tests
  # - on_init validation (no hardware)
  # - export_state_interfaces (no hardware)
  # - lifecycle transitions (real hardware when available, mock mode otherwise)
  # - read() sanity checks (real hardware when available, mock mode otherwise)
  ament_add_gtest(test_hardware_interface
    test/test_hardware_interface.cpp
  )
  target_link_libraries(test_hardware_interface
    ${PROJECT_NAME}
    ${rclcpp_TARGETS}
    hardware_interface::hardware_interface
  )
  target_include_directories(test_hardware_interface PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${BNO055_DRIVER_DIR}
  )

  # Python launch test — full stack against real BNO055 hardware or mock mode.
  # Uses enable_mock_mode:=true when the sensor is absent so CI always runs all tests.
  add_launch_test(test/test_bno055.launch.py)
endif()

ament_package()
