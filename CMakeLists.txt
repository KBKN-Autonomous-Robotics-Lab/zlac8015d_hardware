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
target_include_directories(zlac8015d_hardware PRIVATE include)
target_compile_features(zlac8015d_hardware PUBLIC cxx_std_17)
ament_target_dependencies(zlac8015d_hardware
  rclcpp
  hardware_interface
  pluginlib
)

# Standalone SocketCAN data-acquisition utility for LADRC b0 identification.
# It intentionally does not run through ros2_control, so no controller can
# overwrite the test-current command during the identification experiment.
add_executable(b0_identification
  src/b0_identification.cpp
)
target_compile_features(b0_identification PUBLIC cxx_std_17)

add_executable(b0_identification_moving
  src/b0_identification_moving.cpp
)
target_compile_features(b0_identification_moving PUBLIC cxx_std_17)


# Standalone LADRC step-response validation tool. It talks directly to
# SocketCAN so ros2_control cannot overwrite the test current.
add_executable(ladrc_step_response
  src/ladrc_step_response.cpp
)
target_compile_features(ladrc_step_response PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(hardware_interface zlac8015d_hardware.xml)

add_executable(
  breakaway_current_identification
  src/breakaway_current_identification.cpp
)

target_compile_features(
  breakaway_current_identification
  PUBLIC cxx_std_17
)

add_executable(
  ground_startability_test
  src/ground_startability_test.cpp
)

target_compile_features(
  ground_startability_test
  PUBLIC cxx_std_17
)

install(
  TARGETS zlac8015d_hardware
  DESTINATION lib
)

install(
  TARGETS
    b0_identification
    b0_identification_moving
    ladrc_step_response
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ladrc_step_response
    breakaway_current_identification
    ground_startability_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/analyze_b0.py
    scripts/analyze_ladrc_step.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
