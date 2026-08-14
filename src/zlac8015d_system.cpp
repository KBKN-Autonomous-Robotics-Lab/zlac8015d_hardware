#include "zlac8015d_hardware/zlac8015d_system.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace zlac8015d_hardware
{

namespace
{

// Calculate signed encoder-count difference while handling int32 wrap-around.
int64_t wrapped_count_delta(const int32_t current, const int32_t previous)
{
  int64_t delta =
    static_cast<int64_t>(current) - static_cast<int64_t>(previous);

  constexpr int64_t kEncoderModulo = (int64_t{1} << 32);

  if (delta > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
    delta -= kEncoderModulo;
  } else if (delta < static_cast<int64_t>(std::numeric_limits<int32_t>::min())) {
    delta += kEncoderModulo;
  }

  return delta;
}

}  // namespace

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);

  raw_position_left_ = 0;
  raw_position_right_ = 0;
  last_raw_position_left_ = 0;
  last_raw_position_right_ = 0;
  accumulated_counts_left_ = 0;
  accumulated_counts_right_ = 0;
  position_initialized_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  struct sockaddr_can addr {};
  struct ifreq ifr {};

  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Failed to create CAN socket.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Failed to get CAN interface index for can0.");
    close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(
      can_socket_,
      reinterpret_cast<struct sockaddr *>(&addr),
      sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Failed to bind CAN socket to can0.");
    close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  fcntl(can_socket_, F_SETFL, O_NONBLOCK);

  // Reset encoder-origin state for this activation.
  position_initialized_ = false;
  accumulated_counts_left_ = 0;
  accumulated_counts_right_ = 0;
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;

  // ----------------------------------------------------------------
  // CANopen configuration
  // Assumption: ZLAC8015D Node-ID = 1
  // SDO request COB-ID = 0x601
  // ----------------------------------------------------------------

  // NMT: Pre-Operational
  send_can_frame(0x000, {0x80, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // ----------------------------------------------------------------
  // RPDO1: 60FFh:03 combined target velocity
  //
  // Low  16 bit = Left target velocity
  // High 16 bit = Right target velocity
  // Unit = 0.1 rpm
  // ----------------------------------------------------------------

  // Clear RPDO1 mapping: 1601h:00 = 0
  send_can_frame(
    0x601,
    {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Map 60FFh:03 (32 bit) -> 1601h:01
  // Descriptor: 0x60FF0320
  send_can_frame(
    0x601,
    {0x23, 0x01, 0x16, 0x01, 0x20, 0x03, 0xFF, 0x60});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Enable one mapped object: 1601h:00 = 1
  send_can_frame(
    0x601,
    {0x2F, 0x01, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 200Fh:00 = 1, synchronous left/right control flag
  send_can_frame(
    0x601,
    {0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // ----------------------------------------------------------------
  // TPDO2: 6064h Position actual value
  //
  // TPDO2 COB-ID (Node-ID 1): 0x381
  //
  // byte 0..3 = 6064h:01 Left  [I32, count]
  // byte 4..7 = 6064h:02 Right [I32, count]
  //
  // Transmission:
  //   type 255 (asynchronous)
  //   inhibit time 20 ms
  //   event timer 20 ms
  //
  // Therefore a fresh position sample is produced periodically even
  // while the wheel is stopped, and the PDO rate is limited to ~50 Hz.
  // ----------------------------------------------------------------

  // Clear TPDO2 mapping: 1A02h:00 = 0
  send_can_frame(
    0x601,
    {0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Map 6064h:01 Left actual position, 32 bit
  // Descriptor: 0x60640120
  send_can_frame(
    0x601,
    {0x23, 0x02, 0x1A, 0x01, 0x20, 0x01, 0x64, 0x60});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Map 6064h:02 Right actual position, 32 bit
  // Descriptor: 0x60640220
  send_can_frame(
    0x601,
    {0x23, 0x02, 0x1A, 0x02, 0x20, 0x02, 0x64, 0x60});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // TPDO2 transmission type = 255 (asynchronous / timer capable)
  send_can_frame(
    0x601,
    {0x2F, 0x02, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // TPDO2 inhibit time = 200 * 100 us = 20 ms
  send_can_frame(
    0x601,
    {0x2B, 0x02, 0x18, 0x03, 0xC8, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // TPDO2 event timer = 40 * 500 us = 20 ms
  send_can_frame(
    0x601,
    {0x2B, 0x02, 0x18, 0x05, 0x28, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Enable two mapped objects: 1A02h:00 = 2
  send_can_frame(
    0x601,
    {0x2F, 0x02, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // ----------------------------------------------------------------
  // Start CANopen operation and enable Profile Velocity Mode
  // ----------------------------------------------------------------

  // NMT: Operational
  send_can_frame(0x000, {0x01, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 6060h:00 = 3 -> Profile Velocity Mode
  send_can_frame(
    0x601,
    {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Servo ON: 6040h = 06 -> 07 -> 0F
  send_can_frame(
    0x601,
    {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  send_can_frame(
    0x601,
    {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  send_can_frame(
    0x601,
    {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  RCLCPP_INFO(
    rclcpp::get_logger("Zlac8015dSystemHardware"),
    "Activated CAN communication. 6064h encoder position feedback is configured at 20 ms.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (can_socket_ >= 0) {
    // Servo OFF / shutdown command
    send_can_frame(
      0x601,
      {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00});

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    close(can_socket_);
    can_socket_ = -1;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Zlac8015dSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Zlac8015dSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type Zlac8015dSystemHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  struct can_frame frame {};

  while (::read(can_socket_, &frame, sizeof(struct can_frame)) > 0) {

    // ------------------------------------------------------------
    // TPDO0: 606Ch:03 actual velocity
    //
    // High 16 = Left
    // Low  16 = Right
    // Unit = 0.1 rpm
    // ------------------------------------------------------------
if (frame.can_id == 0x181 && frame.can_dlc >= 4) {
  uint32_t value = 0;
  std::memcpy(&value, frame.data, sizeof(value));

  const int16_t low16 =
    static_cast<int16_t>(value & 0xFFFFu);

  const int16_t high16 =
    static_cast<int16_t>((value >> 16) & 0xFFFFu);

  // 実機で確認された配置
  // Low 16  = Left
  // High 16 = Right
  const int16_t val_l = low16;
  const int16_t val_r = high16;

  // 前進時にROS joint velocityが両輪とも正になるよう符号補正
  hw_velocities_[0] =
    -static_cast<double>(val_l) * RPM01_TO_RAD_PER_SEC;

  hw_velocities_[1] =
    static_cast<double>(val_r) * RPM01_TO_RAD_PER_SEC;
} // ------------------------------------------------------------
    // TPDO1: 6077h:03 actual torque/current
    //
    // Low  16 = Left
    // High 16 = Right
    // Unit = 0.1 A
    // ------------------------------------------------------------
    else if (frame.can_id == 0x281 && frame.can_dlc >= 4) {
      uint32_t value = 0;
      std::memcpy(&value, frame.data, sizeof(value));

      const int16_t val_l =
        static_cast<int16_t>(value & 0xFFFFu);

      const int16_t val_r =
        static_cast<int16_t>((value >> 16) & 0xFFFFu);

      hw_efforts_[0] =
        -static_cast<double>(val_l) * 0.1;

      hw_efforts_[1] =
        static_cast<double>(val_r) * 0.1;
    }

    // ------------------------------------------------------------
    // TPDO2: 6064h actual position
    //
    // byte 0..3 = Left  actual position [I32, count]
    // byte 4..7 = Right actual position [I32, count]
    //
    // Position is unwrapped from the first received sample and converted
    // directly to radians. Velocity integration is no longer used.
    // ------------------------------------------------------------
    else if (frame.can_id == 0x381 && frame.can_dlc >= 8) {
      std::memcpy(
        &raw_position_left_,
        &frame.data[0],
        sizeof(raw_position_left_));

      std::memcpy(
        &raw_position_right_,
        &frame.data[4],
        sizeof(raw_position_right_));

      if (!position_initialized_) {
        last_raw_position_left_ = raw_position_left_;
        last_raw_position_right_ = raw_position_right_;

        accumulated_counts_left_ = 0;
        accumulated_counts_right_ = 0;

        hw_positions_[0] = 0.0;
        hw_positions_[1] = 0.0;

        position_initialized_ = true;

        RCLCPP_INFO(
          rclcpp::get_logger("Zlac8015dSystemHardware"),
          "6064 encoder origin initialized. Raw L: %d, R: %d",
          raw_position_left_,
          raw_position_right_);
      } else {
        const int64_t delta_left =
          wrapped_count_delta(
            raw_position_left_,
            last_raw_position_left_);

        const int64_t delta_right =
          wrapped_count_delta(
            raw_position_right_,
            last_raw_position_right_);

        accumulated_counts_left_ += delta_left;
        accumulated_counts_right_ += delta_right;

        last_raw_position_left_ = raw_position_left_;
        last_raw_position_right_ = raw_position_right_;

        // Forward motion:
        //   left raw count decreases  -> invert sign
        //   right raw count increases -> keep sign
        hw_positions_[0] =
          -static_cast<double>(accumulated_counts_left_) *
          RAD_PER_ENCODER_COUNT;

        hw_positions_[1] =
          static_cast<double>(accumulated_counts_right_) *
          RAD_PER_ENCODER_COUNT;
      }

      // Approximately once per second at 50 Hz.
      static int position_debug_count = 0;
      if (++position_debug_count % 50 == 0) {
        RCLCPP_INFO(
          rclcpp::get_logger("Zlac8015dSystemHardware"),
          "6064 raw L: %d, R: %d | pos L: %.4f rad, R: %.4f rad",
          raw_position_left_,
          raw_position_right_,
          hw_positions_[0],
          hw_positions_[1]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zlac8015dSystemHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  const int16_t cmd_l =
    static_cast<int16_t>(
      -hw_commands_[0] * RAD_PER_SEC_TO_01RPM);

  const int16_t cmd_r =
    static_cast<int16_t>(
      hw_commands_[1] * RAD_PER_SEC_TO_01RPM);

  static int debug_count = 0;

  if (debug_count++ % 50 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "CMD L: %.2f rad/s (%d), R: %.2f rad/s (%d)",
      hw_commands_[0], cmd_l,
      hw_commands_[1], cmd_r);
  }

  // 60FFh:03:
  // Low 16 = Left target velocity
  // High 16 = Right target velocity
  const uint32_t combined_cmd =
    (static_cast<uint32_t>(static_cast<uint16_t>(cmd_r)) << 16) |
    static_cast<uint16_t>(cmd_l);

  struct can_frame frame {};
  frame.can_id = 0x301;
  frame.can_dlc = 4;

  std::memcpy(
    frame.data,
    &combined_cmd,
    sizeof(combined_cmd));

  ::write(
    can_socket_,
    &frame,
    sizeof(struct can_frame));

  return hardware_interface::return_type::OK;
}

bool Zlac8015dSystemHardware::send_can_frame(
  uint32_t can_id,
  const std::vector<uint8_t> & data)
{
  if (can_socket_ < 0 || data.size() > CAN_MAX_DLEN) {
    return false;
  }

  struct can_frame frame {};
  frame.can_id = can_id;
  frame.can_dlc = static_cast<__u8>(data.size());

  std::memcpy(
    frame.data,
    data.data(),
    data.size());

  return ::write(
    can_socket_,
    &frame,
    sizeof(struct can_frame)) > 0;
}

}  // namespace zlac8015d_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  zlac8015d_hardware::Zlac8015dSystemHardware,
  hardware_interface::SystemInterface)
