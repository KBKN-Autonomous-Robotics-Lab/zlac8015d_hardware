#ifndef ZLAC8015D_SYSTEM_HPP_
#define ZLAC8015D_SYSTEM_HPP_

#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

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
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CANソケット関連
  int can_socket_;
  bool send_can_frame(uint32_t can_id, const std::vector<uint8_t>& data);

  // ジョイントの状態とコマンドを保持する変数
  std::vector<double> hw_commands_;   // 目標速度 [rad/s]
  std::vector<double> hw_positions_;  // 実位置 [rad] (積分して算出)
  std::vector<double> hw_velocities_; // 実速度 [rad/s]
  std::vector<double> hw_efforts_;    // 実電流 [A]

  // 定数
  const double RAD_PER_SEC_TO_01RPM = 95.4929658551; // rad/s -> rpm (整数=1RPM単位)
  const double RPM01_TO_RAD_PER_SEC = 0.01047197551; 
};
}  // namespace zlac8015d_hardware

#endif  // ZLAC8015D_SYSTEM_HPP_
