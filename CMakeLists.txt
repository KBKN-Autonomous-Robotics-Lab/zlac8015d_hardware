cmake_minimum_required(VERSION 3.8)
project(zlac8015d_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(zlac8015d_hardware SHARED
  src/zlac8015d_system.cpp
)

target_include_directories(
  zlac8015d_hardware
  PRIVATE include
)

target_compile_features(
  zlac8015d_hardware
  PUBLIC cxx_std_17
)

ament_target_dependencies(
  zlac8015d_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  zlac8015d_hardware.xml
)

install(
  TARGETS zlac8015d_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)



# Standalone SocketCAN step-response acquisition for first-order plant identification.
add_executable(first_order_identification
  src/first_order_identification.cpp
)
target_compile_features(first_order_identification PUBLIC cxx_std_17)

install(
  TARGETS first_order_identification
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/analyze_first_order.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


# Running-wheel current switching experiment
add_executable(running_step_identification
  src/running_step_identification.cpp
)

install(TARGETS
  running_step_identification
  DESTINATION lib/${PROJECT_NAME}
)
