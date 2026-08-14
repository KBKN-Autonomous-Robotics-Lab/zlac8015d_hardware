cmake_minimum_required(VERSION 3.8)
project(zlac8015d_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(zlac8015d_hardware SHARED
  src/zlac8015d_system.cpp
)
target_include_directories(zlac8015d_hardware PRIVATE include)
ament_target_dependencies(zlac8015d_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface zlac8015d_hardware.xml)

install(
  TARGETS zlac8015d_hardware
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_package()
