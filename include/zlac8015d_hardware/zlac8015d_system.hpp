#ifndef ZLAC8015D_SYSTEM_HPP_
#define ZLAC8015D_SYSTEM_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace zlac8015d_hardware
{

class Zlac8015dSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // CAN socket
  int can_socket_{-1};

  bool send_can_frame(
    uint32_t can_id,
    const std::vector<uint8_t> & data);

  // ros2_control command / state
  std::vector<double> hw_commands_;    // target wheel velocity [rad/s]
  std::vector<double> hw_positions_;   // encoder-based wheel position [rad]
  std::vector<double> hw_velocities_;  // actual wheel velocity [rad/s]
  std::vector<double> hw_efforts_;     // actual motor current [A]

  // 6064h Position actual value [count]
  int32_t raw_position_left_{0};
  int32_t raw_position_right_{0};
  int32_t last_raw_position_left_{0};
  int32_t last_raw_position_right_{0};

  // Unwrapped encoder count from the first received 6064h sample.
  int64_t accumulated_counts_left_{0};
  int64_t accumulated_counts_right_{0};
  bool position_initialized_{false};

  // Unit conversion
  // rad/s -> 0.1 rpm
  static constexpr double RAD_PER_SEC_TO_01RPM = 95.4929658551;

  // 0.1 rpm -> rad/s
  static constexpr double RPM01_TO_RAD_PER_SEC = 0.01047197551;

  // ZLLG45ASM200 encoder setting is 1024 lines.
  // The 6064h feedback is treated as 4x quadrature count:
  // 1024 * 4 = 4096 counts / wheel revolution.
  static constexpr double ENCODER_COUNTS_PER_REV = 4096.0;
  static constexpr double TWO_PI = 6.28318530717958647692;
  static constexpr double RAD_PER_ENCODER_COUNT =
    TWO_PI / ENCODER_COUNTS_PER_REV;
};

}  // namespace zlac8015d_hardware

#endif  // ZLAC8015D_SYSTEM_HPP_
