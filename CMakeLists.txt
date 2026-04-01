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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
