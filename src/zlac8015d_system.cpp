#include "zlac8015d_hardware/zlac8015d_system.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>

namespace zlac8015d_hardware
{

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 変数の初期化 (左車輪, 右車輪の2つを想定)
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  struct sockaddr_can addr;
  struct ifreq ifr;

  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(ifr.ifr_name, "can0");
  ioctl(can_socket_, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr));
  fcntl(can_socket_, F_SETFL, O_NONBLOCK);

  // ----------------------------------------------------------------
  // 確実なRPDOマッピングとサーボONシーケンス (SDO生フレーム送信)
  // ----------------------------------------------------------------
  // NMT: Pre-Operational状態へ (マッピング変更のため)
  send_can_frame(0x000, {0x80, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // RPDO1 マッピング無効化 (0x1601 Sub 00 = 0)
  send_can_frame(0x601, {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // RPDO1 割り当て: 0x60FF sub 03 (32bit=0x20) -> 0x60FF0320
  send_can_frame(0x601, {0x23, 0x01, 0x16, 0x01, 0x20, 0x03, 0xFF, 0x60});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // RPDO1 マッピング有効化 (1個)
  send_can_frame(0x601, {0x2F, 0x01, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 同期制御モードを有効化 (200Fh:00 = 1) ※60FF sub03への書き込みに必須
  send_can_frame(0x601, {0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  // NMT: Operational状態へ移行
  send_can_frame(0x000, {0x01, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 動作モード設定: 0x6060 Sub 00 に 3 (Profile Velocity Mode)
  send_can_frame(0x601, {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // サーボONシーケンス
  send_can_frame(0x601, {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  send_can_frame(0x601, {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  send_can_frame(0x601, {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(rclcpp::get_logger("Zlac8015dSystemHardware"), "Activated CAN communication, Mapped RPDO, and Servo ON.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 安全のためのサーボOFF処理 (Control Word: 0x06)
  send_can_frame(0x601, {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  close(can_socket_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Zlac8015dSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Zlac8015dSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type Zlac8015dSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  struct can_frame frame;
  // 受信バッファにある全てのフレームを読み出す
  while (::read(can_socket_, &frame, sizeof(struct can_frame)) > 0) {
    if (frame.can_id == 0x181) { // TPDO0: 結合実速度
      int32_t val_32 = *reinterpret_cast<int32_t*>(frame.data);
      int16_t val_l = val_32 & 0xFFFF;
      int16_t val_r = (val_32 >> 16) & 0xFFFF;

      hw_velocities_[0] = -val_l * RPM01_TO_RAD_PER_SEC; // 左車輪
      hw_velocities_[1] = val_r * RPM01_TO_RAD_PER_SEC; // 右車輪

      // 位置(rad)は速度の積分で近似算出する
      hw_positions_[0] += hw_velocities_[0] * period.seconds();
      hw_positions_[1] += hw_velocities_[1] * period.seconds();
    }
    else if (frame.can_id == 0x281) { // TPDO1: 結合実電流
      int32_t val_32 = *reinterpret_cast<int32_t*>(frame.data);
      int16_t val_l = val_32 & 0xFFFF;
      int16_t val_r = (val_32 >> 16) & 0xFFFF;

      hw_efforts_[0] = -val_l * 0.1; // 左電流(A)
      hw_efforts_[1] = val_r * 0.1; // 右電流(A)
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zlac8015dSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int16_t cmd_l = static_cast<int16_t>(-hw_commands_[0] * RAD_PER_SEC_TO_01RPM);
  int16_t cmd_r = static_cast<int16_t>(hw_commands_[1] * RAD_PER_SEC_TO_01RPM);

  // --- デバッグ出力 (約1秒に1回ターミナルに表示) ---
  static int debug_count = 0;
  if (debug_count++ % 50 == 0) {
      RCLCPP_INFO(rclcpp::get_logger("Zlac8015dSystemHardware"), 
        "CMD L: %.2f rad/s (%d), R: %.2f rad/s (%d)", 
        hw_commands_[0], cmd_l, hw_commands_[1], cmd_r);
  }

  uint32_t combined_cmd = ((uint32_t)(uint16_t)cmd_r << 16) | (uint16_t)cmd_l;

  struct can_frame frame;
  frame.can_id = 0x301;
  frame.can_dlc = 4;
  std::memcpy(frame.data, &combined_cmd, sizeof(combined_cmd));

  ::write(can_socket_, &frame, sizeof(struct can_frame));

  return hardware_interface::return_type::OK;
}

// ヘルパー関数: CANフレームの送信
bool Zlac8015dSystemHardware::send_can_frame(uint32_t can_id, const std::vector<uint8_t>& data) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data.size();
    std::memcpy(frame.data, data.data(), data.size());
    return ::write(can_socket_, &frame, sizeof(struct can_frame)) > 0;
}

}  // namespace zlac8015d_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  zlac8015d_hardware::Zlac8015dSystemHardware, hardware_interface::SystemInterface)
