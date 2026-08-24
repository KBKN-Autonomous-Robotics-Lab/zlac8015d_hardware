#include "zlac8015d_hardware/zlac8015d_system.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace zlac8015d_hardware
{

namespace
{

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

double get_double_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const double default_value)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return std::stod(it->second);
}

int get_int_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const int default_value)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return std::stoi(it->second);
}

std::string get_string_parameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_value)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

void sleep_after_sdo()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

}  // namespace

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Expected exactly 2 wheel joints, got %zu.", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);

  try {
    can_interface_ = get_string_parameter(info_, "can_interface", "can0");
    const int node_id = get_int_parameter(info_, "node_id", 1);
    if (node_id < 1 || node_id > 127) {
      throw std::runtime_error("node_id must be in [1, 127]");
    }
    node_id_ = static_cast<uint8_t>(node_id);

    const std::string mode =
      get_string_parameter(info_, "control_mode", "velocity");
    if (mode == "velocity") {
      control_mode_ = ControlMode::VELOCITY;
    } else if (mode == "ladrc_torque") {
      control_mode_ = ControlMode::LADRC_TORQUE;
    } else {
      throw std::runtime_error(
        "control_mode must be 'velocity' or 'ladrc_torque'");
    }

    left_ladrc_.b0 = get_double_parameter(info_, "ladrc_b0_left", 1.0);
    right_ladrc_.b0 = get_double_parameter(info_, "ladrc_b0_right", 1.0);
    controller_bandwidth_ = get_double_parameter(info_, "ladrc_wc", 5.0);
    observer_bandwidth_ = get_double_parameter(info_, "ladrc_wo", 20.0);
    current_limit_a_ = get_double_parameter(info_, "current_limit_a", 2.0);
    current_rate_limit_a_per_s_ =
      get_double_parameter(info_, "current_rate_limit_a_per_s", 20.0);
    feedback_timeout_s_ =
      get_double_parameter(info_, "feedback_timeout_ms", 30.0) / 1000.0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Invalid hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (left_ladrc_.b0 <= 0.0 || right_ladrc_.b0 <= 0.0 ||
      controller_bandwidth_ <= 0.0 || observer_bandwidth_ <= 0.0 ||
      current_limit_a_ <= 0.0 || current_rate_limit_a_per_s_ <= 0.0 ||
      feedback_timeout_s_ <= 0.0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "LADRC/current/watchdog parameters must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  raw_position_left_ = 0;
  raw_position_right_ = 0;
  last_raw_position_left_ = 0;
  last_raw_position_right_ = 0;
  accumulated_counts_left_ = 0;
  accumulated_counts_right_ = 0;
  position_initialized_ = false;
  velocity_feedback_received_ = false;
  current_feedback_received_ = false;
  feedback_timeout_reported_ = false;
  left_ladrc_ = LadrcState{0.0, 0.0, left_ladrc_.b0, 0.0, false};
  right_ladrc_ = LadrcState{0.0, 0.0, right_ladrc_.b0, 0.0, false};

  RCLCPP_INFO(
    rclcpp::get_logger("Zlac8015dSystemHardware"),
    "Configured mode=%s, CAN=%s, node=%u, wc=%.3f, wo=%.3f, Imax=%.3f A",
    control_mode_ == ControlMode::VELOCITY ? "velocity" : "ladrc_torque",
    can_interface_.c_str(),
    static_cast<unsigned>(node_id_),
    controller_bandwidth_, observer_bandwidth_, current_limit_a_);

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

  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Failed to get CAN interface index for %s.", can_interface_.c_str());
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
      "Failed to bind CAN socket to %s.", can_interface_.c_str());
    close(can_socket_);
    can_socket_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  fcntl(can_socket_, F_SETFL, O_NONBLOCK);

  position_initialized_ = false;
  accumulated_counts_left_ = 0;
  accumulated_counts_right_ = 0;
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;
  velocity_feedback_received_ = false;
  current_feedback_received_ = false;
  feedback_timeout_reported_ = false;
  left_ladrc_.z1 = 0.0;
  left_ladrc_.z2 = 0.0;
  left_ladrc_.last_current_cmd = 0.0;
  left_ladrc_.initialized = false;
  right_ladrc_.z1 = 0.0;
  right_ladrc_.z2 = 0.0;
  right_ladrc_.last_current_cmd = 0.0;
  right_ladrc_.initialized = false;

  const uint32_t sdo_id = 0x600u + node_id_;

  // NMT: Pre-Operational for this node only.
  send_can_frame(0x000, {0x80, node_id_});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  if (!configure_feedback_pdos()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "Failed to send feedback PDO configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (control_mode_ == ControlMode::VELOCITY) {
    if (!configure_velocity_rpdo()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Zlac8015dSystemHardware"),
        "Failed to configure velocity RPDO.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    if (!configure_torque_rpdo()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Zlac8015dSystemHardware"),
        "Failed to configure torque RPDO.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 200Fh:00 = 1: synchronized left/right command update inside the driver.
  send_can_frame(
    sdo_id,
    {0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00});
  sleep_after_sdo();

  // NMT: Operational.
  send_can_frame(0x000, {0x01, node_id_});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // 6060h: Profile Velocity (3) or Profile Torque (4).
  const uint8_t operation_mode =
    control_mode_ == ControlMode::VELOCITY ? 0x03 : 0x04;
  send_can_frame(
    sdo_id,
    {0x2F, 0x60, 0x60, 0x00, operation_mode, 0x00, 0x00, 0x00});
  sleep_after_sdo();

  send_zero_command();

  // CiA402 Servo ON: 6040h = 06 -> 07 -> 0F.
  send_can_frame(
    sdo_id,
    {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00});
  sleep_after_sdo();
  send_can_frame(
    sdo_id,
    {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00});
  sleep_after_sdo();
  send_can_frame(
    sdo_id,
    {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00});
  sleep_after_sdo();

  RCLCPP_INFO(
    rclcpp::get_logger("Zlac8015dSystemHardware"),
    "Activated %s mode. Velocity/current TPDO period=5 ms, position TPDO period=20 ms.",
    control_mode_ == ControlMode::VELOCITY ? "velocity" : "LADRC torque");

  if (control_mode_ == ControlMode::LADRC_TORQUE) {
    RCLCPP_INFO(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "LADRC: b0L=%.6f, b0R=%.6f, wc=%.3f, wo=%.3f, Imax=%.3f A, dI/dt=%.3f A/s",
      left_ladrc_.b0, right_ladrc_.b0,
      controller_bandwidth_, observer_bandwidth_,
      current_limit_a_, current_rate_limit_a_per_s_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Zlac8015dSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (can_socket_ >= 0) {
    send_zero_command();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const uint32_t sdo_id = 0x600u + node_id_;
    send_can_frame(
      sdo_id,
      {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00});
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

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
    // Keep a velocity command interface even in LADRC torque mode.
    // diff_drive_controller continues to generate wheel-speed references;
    // the hardware plugin converts them to motor-current commands.
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

  const uint32_t tpdo0_id = 0x180u + node_id_;
  const uint32_t tpdo1_id = 0x280u + node_id_;
  const uint32_t tpdo2_id = 0x380u + node_id_;

  while (::read(can_socket_, &frame, sizeof(struct can_frame)) > 0) {
    // TPDO0: 606Ch:03 actual velocity, unit 0.1 rpm.
    if (frame.can_id == tpdo0_id && frame.can_dlc >= 4) {
      uint32_t value = 0;
      std::memcpy(&value, frame.data, sizeof(value));

      // Preserve the byte ordering verified on the current robot/firmware:
      // Low 16 = Left, High 16 = Right.
      const int16_t val_l =
        static_cast<int16_t>(value & 0xFFFFu);
      const int16_t val_r =
        static_cast<int16_t>((value >> 16) & 0xFFFFu);

      // ROS convention: both wheel velocities are positive for forward motion.
      hw_velocities_[0] =
        -static_cast<double>(val_l) * RPM01_TO_RAD_PER_SEC;
      hw_velocities_[1] =
        static_cast<double>(val_r) * RPM01_TO_RAD_PER_SEC;

      velocity_feedback_received_ = true;
      last_velocity_feedback_time_ = std::chrono::steady_clock::now();
      feedback_timeout_reported_ = false;
    }
    // TPDO1: 6077h:03 actual current, unit 0.1 A.
    else if (frame.can_id == tpdo1_id && frame.can_dlc >= 4) {
      uint32_t value = 0;
      std::memcpy(&value, frame.data, sizeof(value));

      const int16_t val_l =
        static_cast<int16_t>(value & 0xFFFFu);
      const int16_t val_r =
        static_cast<int16_t>((value >> 16) & 0xFFFFu);

      // Same ROS-forward sign convention as velocity.
      hw_efforts_[0] = -static_cast<double>(val_l) * 0.1;
      hw_efforts_[1] = static_cast<double>(val_r) * 0.1;
      current_feedback_received_ = true;
    }
    // TPDO2: 6064h actual position, left/right I32 encoder counts.
    else if (frame.can_id == tpdo2_id && frame.can_dlc >= 8) {
      std::memcpy(
        &raw_position_left_, &frame.data[0], sizeof(raw_position_left_));
      std::memcpy(
        &raw_position_right_, &frame.data[4], sizeof(raw_position_right_));

      if (!position_initialized_) {
        last_raw_position_left_ = raw_position_left_;
        last_raw_position_right_ = raw_position_right_;
        accumulated_counts_left_ = 0;
        accumulated_counts_right_ = 0;
        hw_positions_[0] = 0.0;
        hw_positions_[1] = 0.0;
        position_initialized_ = true;
      } else {
        accumulated_counts_left_ += wrapped_count_delta(
          raw_position_left_, last_raw_position_left_);
        accumulated_counts_right_ += wrapped_count_delta(
          raw_position_right_, last_raw_position_right_);

        last_raw_position_left_ = raw_position_left_;
        last_raw_position_right_ = raw_position_right_;

        hw_positions_[0] =
          -static_cast<double>(accumulated_counts_left_) * RAD_PER_ENCODER_COUNT;
        hw_positions_[1] =
          static_cast<double>(accumulated_counts_right_) * RAD_PER_ENCODER_COUNT;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zlac8015dSystemHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (can_socket_ < 0) {
    return hardware_interface::return_type::ERROR;
  }

  if (control_mode_ == ControlMode::VELOCITY) {
    const int16_t cmd_l = static_cast<int16_t>(
      clamp(-hw_commands_[0] * RAD_PER_SEC_TO_01RPM, -32768.0, 32767.0));
    const int16_t cmd_r = static_cast<int16_t>(
      clamp(hw_commands_[1] * RAD_PER_SEC_TO_01RPM, -32768.0, 32767.0));

    const uint32_t combined_cmd =
      (static_cast<uint32_t>(static_cast<uint16_t>(cmd_r)) << 16) |
      static_cast<uint16_t>(cmd_l);

    struct can_frame frame {};
    frame.can_id = 0x300u + node_id_;
    frame.can_dlc = 4;
    std::memcpy(frame.data, &combined_cmd, sizeof(combined_cmd));

    if (::write(can_socket_, &frame, sizeof(struct can_frame)) <= 0) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  // LADRC torque mode requires fresh velocity feedback.
  if (!velocity_feedback_received_) {
    send_zero_command();
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();
  const double feedback_age =
    std::chrono::duration<double>(now - last_velocity_feedback_time_).count();
  if (feedback_age > feedback_timeout_s_) {
    send_zero_command();
    if (!feedback_timeout_reported_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Zlac8015dSystemHardware"),
        "Velocity feedback timeout: %.1f ms > %.1f ms. Torque command forced to zero.",
        feedback_age * 1000.0, feedback_timeout_s_ * 1000.0);
      feedback_timeout_reported_ = true;
    }
    return hardware_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    dt = 0.005;
  }
  // Protect the explicit Euler observer from a large scheduler hiccup.
  dt = clamp(dt, 0.001, 0.020);

  const double current_l_measured =
    current_feedback_received_ ? hw_efforts_[0] : left_ladrc_.last_current_cmd;
  const double current_r_measured =
    current_feedback_received_ ? hw_efforts_[1] : right_ladrc_.last_current_cmd;

  const double cmd_current_l = compute_ladrc_current(
    left_ladrc_, hw_commands_[0], hw_velocities_[0], current_l_measured, dt);
  const double cmd_current_r = compute_ladrc_current(
    right_ladrc_, hw_commands_[1], hw_velocities_[1], current_r_measured, dt);

  // ZLAC command convention on this robot: left sign is opposite to ROS-forward.
  const int16_t cmd_l_ma = static_cast<int16_t>(
    std::lround(clamp(-cmd_current_l * 1000.0, -30000.0, 30000.0)));
  const int16_t cmd_r_ma = static_cast<int16_t>(
    std::lround(clamp(cmd_current_r * 1000.0, -30000.0, 30000.0)));

  const uint32_t combined_cmd =
    (static_cast<uint32_t>(static_cast<uint16_t>(cmd_r_ma)) << 16) |
    static_cast<uint16_t>(cmd_l_ma);

  struct can_frame frame {};
  frame.can_id = 0x300u + node_id_;
  frame.can_dlc = 4;
  std::memcpy(frame.data, &combined_cmd, sizeof(combined_cmd));

  if (::write(can_socket_, &frame, sizeof(struct can_frame)) <= 0) {
    return hardware_interface::return_type::ERROR;
  }

  static int debug_count = 0;
  if (++debug_count % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("Zlac8015dSystemHardware"),
      "LADRC ref[L,R]=[%.2f, %.2f] rad/s vel=[%.2f, %.2f] A_cmd=[%.2f, %.2f] z2=[%.2f, %.2f]",
      hw_commands_[0], hw_commands_[1],
      hw_velocities_[0], hw_velocities_[1],
      cmd_current_l, cmd_current_r,
      left_ladrc_.z2, right_ladrc_.z2);
  }

  return hardware_interface::return_type::OK;
}

double Zlac8015dSystemHardware::compute_ladrc_current(
  LadrcState & state,
  const double velocity_reference,
  const double velocity_measured,
  const double current_measured,
  const double dt)
{
  if (!state.initialized) {
    state.z1 = velocity_measured;
    state.z2 = 0.0;
    state.last_current_cmd = 0.0;
    state.initialized = true;
  }

  const double observer_error = velocity_measured - state.z1;
  const double beta1 = 2.0 * observer_bandwidth_;
  const double beta2 = observer_bandwidth_ * observer_bandwidth_;

  // Explicit-Euler LESO for: omega_dot = f + b0 * I.
  const double z1_dot =
    state.z2 + state.b0 * current_measured + beta1 * observer_error;
  const double z2_dot = beta2 * observer_error;
  state.z1 += dt * z1_dot;
  state.z2 += dt * z2_dot;

  const double desired_acceleration =
    controller_bandwidth_ * (velocity_reference - state.z1);
  double current_command =
    (desired_acceleration - state.z2) / state.b0;

  current_command = clamp(current_command, -current_limit_a_, current_limit_a_);

  const double max_delta_current = current_rate_limit_a_per_s_ * dt;
  current_command = clamp(
    current_command,
    state.last_current_cmd - max_delta_current,
    state.last_current_cmd + max_delta_current);

  state.last_current_cmd = current_command;
  return current_command;
}

bool Zlac8015dSystemHardware::configure_feedback_pdos()
{
  const uint32_t sdo_id = 0x600u + node_id_;

  // TPDO0 (0x180 + node): 606Ch:03 combined actual velocity, 32 bit.
  if (!send_can_frame(sdo_id, {0x2F, 0x00, 0x1A, 0x00, 0, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x23, 0x00, 0x1A, 0x01, 0x20, 0x03, 0x6C, 0x60})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x00, 0x18, 0x02, 0xFF, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  // Inhibit: 50 * 100 us = 5 ms.
  if (!send_can_frame(sdo_id, {0x2B, 0x00, 0x18, 0x03, 0x32, 0x00, 0, 0})) {return false;}
  sleep_after_sdo();
  // Event timer: 10 * 500 us = 5 ms.
  if (!send_can_frame(sdo_id, {0x2B, 0x00, 0x18, 0x05, 0x0A, 0x00, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x00, 0x1A, 0x00, 0x01, 0, 0, 0})) {return false;}
  sleep_after_sdo();

  // TPDO1 (0x280 + node): 6077h:03 combined actual current, 32 bit.
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x1A, 0x00, 0, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x23, 0x01, 0x1A, 0x01, 0x20, 0x03, 0x77, 0x60})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x18, 0x02, 0xFF, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2B, 0x01, 0x18, 0x03, 0x32, 0x00, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2B, 0x01, 0x18, 0x05, 0x0A, 0x00, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x1A, 0x00, 0x01, 0, 0, 0})) {return false;}
  sleep_after_sdo();

  // TPDO2 (0x380 + node): 6064h:01 and :02 actual positions, 20 ms.
  if (!send_can_frame(sdo_id, {0x2F, 0x02, 0x1A, 0x00, 0, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x23, 0x02, 0x1A, 0x01, 0x20, 0x01, 0x64, 0x60})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x23, 0x02, 0x1A, 0x02, 0x20, 0x02, 0x64, 0x60})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x02, 0x18, 0x02, 0xFF, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2B, 0x02, 0x18, 0x03, 0xC8, 0x00, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2B, 0x02, 0x18, 0x05, 0x28, 0x00, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x02, 0x1A, 0x00, 0x02, 0, 0, 0})) {return false;}
  sleep_after_sdo();

  return true;
}

bool Zlac8015dSystemHardware::configure_velocity_rpdo()
{
  const uint32_t sdo_id = 0x600u + node_id_;

  // RPDO1 (0x300 + node): 60FFh:03 combined target velocity, 32 bit.
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x16, 0x00, 0, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x23, 0x01, 0x16, 0x01, 0x20, 0x03, 0xFF, 0x60})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x16, 0x00, 0x01, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  return true;
}

bool Zlac8015dSystemHardware::configure_torque_rpdo()
{
  const uint32_t sdo_id = 0x600u + node_id_;

  // RPDO1 (0x300 + node): 6071h:03 combined target torque/current, 32 bit.
  // Low 16 = left [mA], High 16 = right [mA].
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x16, 0x00, 0, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x23, 0x01, 0x16, 0x01, 0x20, 0x03, 0x71, 0x60})) {return false;}
  sleep_after_sdo();
  if (!send_can_frame(sdo_id, {0x2F, 0x01, 0x16, 0x00, 0x01, 0, 0, 0})) {return false;}
  sleep_after_sdo();
  return true;
}

void Zlac8015dSystemHardware::send_zero_command()
{
  if (can_socket_ < 0) {
    return;
  }

  const uint32_t zero = 0;
  struct can_frame frame {};
  frame.can_id = 0x300u + node_id_;
  frame.can_dlc = 4;
  std::memcpy(frame.data, &zero, sizeof(zero));
  ::write(can_socket_, &frame, sizeof(struct can_frame));

  left_ladrc_.last_current_cmd = 0.0;
  right_ladrc_.last_current_cmd = 0.0;
}

double Zlac8015dSystemHardware::clamp(
  const double value,
  const double lower,
  const double upper)
{
  return std::max(lower, std::min(value, upper));
}

bool Zlac8015dSystemHardware::send_can_frame(
  const uint32_t can_id,
  const std::vector<uint8_t> & data)
{
  if (can_socket_ < 0 || data.size() > CAN_MAX_DLEN) {
    return false;
  }

  struct can_frame frame {};
  frame.can_id = can_id;
  frame.can_dlc = static_cast<__u8>(data.size());
  std::memcpy(frame.data, data.data(), data.size());

  return ::write(can_socket_, &frame, sizeof(struct can_frame)) > 0;
}

}  // namespace zlac8015d_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  zlac8015d_hardware::Zlac8015dSystemHardware,
  hardware_interface::SystemInterface)
