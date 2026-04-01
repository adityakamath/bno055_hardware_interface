cmake_minimum_required(VERSION 3.16)
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

install(TARGETS ${PROJECT_NAME} bno055_driver bno055_i2c
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
  # Disable linters that would recurse into the external/ Bosch vendor directory
  # and the Bosch driver's non-standard style conventions.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # C++ unit / integration tests
  # - on_init validation (no hardware)
  # - export_state_interfaces (no hardware)
  # - lifecycle transitions (requires BNO055 on /dev/i2c-1, skips gracefully)
  # - read() sanity checks (requires BNO055 on /dev/i2c-1, skips gracefully)
  ament_add_gtest(test_hardware_interface
    test/test_hardware_interface.cpp
  )
  target_link_libraries(test_hardware_interface
    ${PROJECT_NAME}
    ${rclcpp_TARGETS}
    hardware_interface::hardware_interface
    i2c
  )
  target_include_directories(test_hardware_interface PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${BNO055_DRIVER_DIR}
  )

  # Python launch test — full stack against real BNO055 hardware.
  # Skips all test cases when the sensor is absent so CI stays green.
  add_launch_test(test/test_bno055.launch.py)
endif()

ament_package()
