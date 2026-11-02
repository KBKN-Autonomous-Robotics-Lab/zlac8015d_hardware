#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr double kRpm01ToRadPerSec = 0.010471975511966;
constexpr double kEncoderCountsPerRev = 4096.0;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kRadPerEncoderCount =
  kTwoPi / kEncoderCountsPerRev;

std::atomic<bool> g_keep_running{true};

void signal_handler(int)
{
  g_keep_running.store(false);
}

struct Options
{
  std::string can_interface{"can0"};
  int node_id{1};

  // Running-wheel step experiment
  double initial_current_a{2.0};
  double switched_current_a{3.0};
  double switch_speed_rad_s{8.0};
  int repeats{5};

  // Safety maximum duration.
  // Normally the trial ends first at speed_limit_rad_s.
  double step_s{5.0};

  double sample_hz{200.0};
  double max_current_a{3.0};

  // Identification operating range
  double speed_limit_rad_s{15.0};

  // Stop check between trials
  double stop_threshold_rad_s{0.15};
  double stop_hold_s{0.50};
  double stop_timeout_s{20.0};

  bool run{false};

  std::string output{"running_step_2A_3A.csv"};
};

struct Feedback
{
  double left_velocity{0.0};
  double right_velocity{0.0};

  double left_current{0.0};
  double right_current{0.0};

  double left_position{0.0};
  double right_position{0.0};

  bool velocity_valid{false};
  bool current_valid{false};
  bool position_valid{false};
  std::chrono::steady_clock::time_point last_velocity_rx{};
  std::chrono::steady_clock::time_point last_current_rx{};

  int32_t last_raw_left{0};
  int32_t last_raw_right{0};

  int64_t accumulated_left{0};
  int64_t accumulated_right{0};
};

int64_t wrapped_count_delta(
  const int32_t current,
  const int32_t previous)
{
  int64_t delta =
    static_cast<int64_t>(current) -
    static_cast<int64_t>(previous);

  constexpr int64_t modulo =
    (int64_t{1} << 32);

  if (delta > INT32_MAX) {
    delta -= modulo;
  } else if (delta < INT32_MIN) {
    delta += modulo;
  }

  return delta;
}

std::vector<double> parse_currents(
  const std::string & text)
{
  std::vector<double> values;

  std::stringstream ss(text);
  std::string token;

  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }

  if (values.empty()) {
    throw std::runtime_error(
      "--currents must contain at least one value");
  }

  return values;
}

void print_usage(const char * argv0)
{
  std::cout << "Usage: " << argv0 << " --run [options]\n"
    << "Running-wheel current switching experiment.\n"
    << "Motor will move. Keep wheels lifted; do not run another motor controller.\n"
    << "  --can can0\n  --node 1\n"
    << "  --initial-current 2.0\n  --switched-current 3.0\n"
    << "  --switch-speed 8.0   (both wheels must reach threshold)\n"
    << "  --speed-limit 15.0  (cutoff if either reaches threshold)\n"
    << "  --repeats 5\n  --step 2.0  (max acceleration time)\n"
    << "  --sample-hz 200\n  --max-current 3.0\n"
    << "  --stop-threshold 0.15\n  --stop-hold 0.50\n"
    << "  --stop-timeout 20.0\n  --output FILE.csv\n  --run\n";
}

Options parse_options(
  int argc,
  char ** argv)
{
  Options opt;

  for (int i = 1; i < argc; ++i) {

    const std::string arg = argv[i];

    auto require_value =
      [&](const std::string & name) -> std::string
      {
        if (i + 1 >= argc) {
          throw std::runtime_error(
            "Missing value after " + name);
        }

        return argv[++i];
      };

    if (arg == "--can") {
      opt.can_interface = require_value(arg);

    } else if (arg == "--node") {
      opt.node_id =
        std::stoi(require_value(arg));

    } else if (arg == "--initial-current") {
      opt.initial_current_a = std::stod(require_value(arg));

    } else if (arg == "--switched-current") {
      opt.switched_current_a = std::stod(require_value(arg));

    } else if (arg == "--switch-speed") {
      opt.switch_speed_rad_s = std::stod(require_value(arg));

    } else if (arg == "--repeats") {
      opt.repeats =
        std::stoi(require_value(arg));

    } else if (arg == "--step") {
      opt.step_s =
        std::stod(require_value(arg));

    } else if (arg == "--speed-limit") {
      opt.speed_limit_rad_s =
        std::stod(require_value(arg));

    } else if (arg == "--sample-hz") {
      opt.sample_hz =
        std::stod(require_value(arg));

    } else if (arg == "--max-current") {
      opt.max_current_a =
        std::stod(require_value(arg));

    } else if (arg == "--stop-threshold") {
      opt.stop_threshold_rad_s =
        std::stod(require_value(arg));

    } else if (arg == "--stop-hold") {
      opt.stop_hold_s =
        std::stod(require_value(arg));

    } else if (arg == "--stop-timeout") {
      opt.stop_timeout_s =
        std::stod(require_value(arg));

    } else if (arg == "--output") {
      opt.output =
        require_value(arg);

    } else if (arg == "--run") {
      opt.run = true;

    } else if (
      arg == "--help" ||
      arg == "-h")
    {
      print_usage(argv[0]);
      std::exit(0);

    } else {
      throw std::runtime_error(
        "Unknown argument: " + arg);
    }
  }

  if (
    opt.node_id < 1 ||
    opt.node_id > 127 ||
    opt.repeats <= 0 ||
    opt.step_s <= 0.0 ||
    opt.initial_current_a <= 0.0 ||
    opt.switched_current_a <= opt.initial_current_a ||
    opt.switched_current_a > opt.max_current_a ||
    opt.initial_current_a > opt.max_current_a ||
    opt.switch_speed_rad_s <= 0.0 ||
    opt.switch_speed_rad_s >= opt.speed_limit_rad_s ||
    opt.sample_hz <= 0.0 ||
    opt.max_current_a <= 0.0 ||
    opt.speed_limit_rad_s <= 0.0 ||
    opt.stop_threshold_rad_s <= 0.0 ||
    opt.stop_hold_s <= 0.0 ||
    opt.stop_timeout_s <= opt.stop_hold_s)
  {
    throw std::runtime_error(
      "Invalid numeric option");
  }

  return opt;
}

class ZlacCan
{
public:

  ZlacCan(
    std::string interface_name,
    int node_id)
  : interface_name_(
      std::move(interface_name)),
    node_id_(node_id)
  {
  }

  ~ZlacCan()
  {
    if (socket_ >= 0) {

      try {
        command_current(0.0);

        std::this_thread::sleep_for(
          std::chrono::milliseconds(20));

        sdo_write_u16(
          0x6040,
          0x00,
          0x0006);

      } catch (...) {
      }

      ::close(socket_);
    }
  }

  void open_socket()
  {
    socket_ =
      ::socket(
        PF_CAN,
        SOCK_RAW,
        CAN_RAW);

    if (socket_ < 0) {
      throw std::runtime_error(
        "Failed to create CAN socket");
    }

    struct ifreq ifr {};

    std::strncpy(
      ifr.ifr_name,
      interface_name_.c_str(),
      IFNAMSIZ - 1);

    if (
      ::ioctl(
        socket_,
        SIOCGIFINDEX,
        &ifr) < 0)
    {
      throw std::runtime_error(
        "Failed to get interface index for " +
        interface_name_);
    }

    struct sockaddr_can addr {};

    addr.can_family =
      AF_CAN;

    addr.can_ifindex =
      ifr.ifr_ifindex;

    if (
      ::bind(
        socket_,
        reinterpret_cast<sockaddr *>(&addr),
        sizeof(addr)) < 0)
    {
      throw std::runtime_error(
        "Failed to bind CAN socket to " +
        interface_name_);
    }

    const int flags =
      ::fcntl(
        socket_,
        F_GETFL,
        0);

    if (flags >= 0) {
      ::fcntl(
        socket_,
        F_SETFL,
        flags | O_NONBLOCK);
    }
  }

  void configure_for_identification()
  {
    // Pre-operational
    nmt(0x80);

    std::this_thread::sleep_for(
      std::chrono::milliseconds(20));

    configure_feedback_pdos();

    // Asynchronous left/right target-current control.
    // 6071h:01 = left
    // 6071h:02 = right
    sdo_write_u16(
      0x200F,
      0x00,
      0);

    // Torque slope: 10000 mA/s
    sdo_write_u32(
      0x6087,
      0x01,
      10000u);

    sdo_write_u32(
      0x6087,
      0x02,
      10000u);

    // Operational
    nmt(0x01);

    std::this_thread::sleep_for(
      std::chrono::milliseconds(20));

    // Profile Torque Mode
    sdo_write_u8(
      0x6060,
      0x00,
      4);

    command_current(0.0);

    // CiA402:
    // shutdown -> switch on -> enable operation
    sdo_write_u16(
      0x6040,
      0x00,
      0x0006);

    sdo_write_u16(
      0x6040,
      0x00,
      0x0007);

    sdo_write_u16(
      0x6040,
      0x00,
      0x000F);
  }

  void command_current(
    const double ros_forward_current_a)
  {
    // 6071h:01 / :02
    // unit = mA
    //
    // Left driver sign is opposite
    // to ROS-forward convention.

    const int16_t left_ma =
      static_cast<int16_t>(
        std::lround(
          std::clamp(
            -ros_forward_current_a * 1000.0,
            -30000.0,
            30000.0)));

    const int16_t right_ma =
      static_cast<int16_t>(
        std::lround(
          std::clamp(
            ros_forward_current_a * 1000.0,
            -30000.0,
            30000.0)));

    sdo_write_u16(
      0x6071,
      0x01,
      static_cast<uint16_t>(left_ma));

    sdo_write_u16(
      0x6071,
      0x02,
      static_cast<uint16_t>(right_ma));
  }

  void drain_feedback(
    Feedback & fb)
  {
    struct can_frame frame {};

    while (
      ::read(
        socket_,
        &frame,
        sizeof(frame)) > 0)
    {
      const uint32_t tpdo0 =
        0x180u +
        static_cast<uint32_t>(
          node_id_);

      const uint32_t tpdo1 =
        0x280u +
        static_cast<uint32_t>(
          node_id_);

      const uint32_t tpdo2 =
        0x380u +
        static_cast<uint32_t>(
          node_id_);

      const uint32_t clean_id =
        frame.can_id &
        CAN_SFF_MASK;

      if (
        clean_id == tpdo0 &&
        frame.can_dlc >= 8)
      {
        int32_t raw_left = 0;
        int32_t raw_right = 0;

        std::memcpy(
          &raw_left,
          &frame.data[0],
          sizeof(raw_left));

        std::memcpy(
          &raw_right,
          &frame.data[4],
          sizeof(raw_right));

        fb.left_velocity =
          -static_cast<double>(
            raw_left) *
          kRpm01ToRadPerSec;

        fb.right_velocity =
          static_cast<double>(
            raw_right) *
          kRpm01ToRadPerSec;

        fb.velocity_valid = true;
        fb.last_velocity_rx = std::chrono::steady_clock::now();
      }

      else if (
        clean_id == tpdo1 &&
        frame.can_dlc >= 4)
      {
        int16_t raw_left = 0;
        int16_t raw_right = 0;

        std::memcpy(
          &raw_left,
          &frame.data[0],
          sizeof(raw_left));

        std::memcpy(
          &raw_right,
          &frame.data[2],
          sizeof(raw_right));

        fb.left_current =
          -static_cast<double>(
            raw_left) * 0.1;

        fb.right_current =
          static_cast<double>(
            raw_right) * 0.1;

        fb.current_valid = true;
        fb.last_current_rx = std::chrono::steady_clock::now();
      }

      else if (
        clean_id == tpdo2 &&
        frame.can_dlc >= 8)
      {
        int32_t raw_left = 0;
        int32_t raw_right = 0;

        std::memcpy(
          &raw_left,
          &frame.data[0],
          sizeof(raw_left));

        std::memcpy(
          &raw_right,
          &frame.data[4],
          sizeof(raw_right));

        if (!fb.position_valid) {

          fb.last_raw_left =
            raw_left;

          fb.last_raw_right =
            raw_right;

          fb.accumulated_left = 0;
          fb.accumulated_right = 0;

          fb.left_position = 0.0;
          fb.right_position = 0.0;

          fb.position_valid = true;

        } else {

          fb.accumulated_left +=
            wrapped_count_delta(
              raw_left,
              fb.last_raw_left);

          fb.accumulated_right +=
            wrapped_count_delta(
              raw_right,
              fb.last_raw_right);

          fb.last_raw_left =
            raw_left;

          fb.last_raw_right =
            raw_right;

          fb.left_position =
            -static_cast<double>(
              fb.accumulated_left) *
            kRadPerEncoderCount;

          fb.right_position =
            static_cast<double>(
              fb.accumulated_right) *
            kRadPerEncoderCount;
        }
      }
    }
  }

  void shutdown()
  {
    if (socket_ < 0) {
      return;
    }

    command_current(0.0);

    std::this_thread::sleep_for(
      std::chrono::milliseconds(20));

    sdo_write_u16(
      0x6040,
      0x00,
      0x0006);
  }

private:

  void send_frame(
    uint32_t can_id,
    const uint8_t * data,
    size_t size)
  {
    struct can_frame frame {};

    frame.can_id =
      can_id;

    frame.can_dlc =
      static_cast<__u8>(size);

    std::memcpy(
      frame.data,
      data,
      size);

    if (
      ::write(
        socket_,
        &frame,
        sizeof(frame)) <= 0)
    {
      throw std::runtime_error(
        "CAN write failed");
    }
  }

  void nmt(
    uint8_t command)
  {
    const uint8_t data[2] =
    {
      command,
      static_cast<uint8_t>(
        node_id_)
    };

    send_frame(
      0x000,
      data,
      2);
  }

  void sdo_write(
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    int bytes)
  {
    uint8_t command =
      0x23;

    if (bytes == 1) {
      command = 0x2F;

    } else if (bytes == 2) {
      command = 0x2B;

    } else if (bytes != 4) {
      throw std::runtime_error(
        "Unsupported SDO write size");
    }

    uint8_t data[8]{};

    data[0] = command;

    data[1] =
      static_cast<uint8_t>(
        index & 0xFFu);

    data[2] =
      static_cast<uint8_t>(
        (index >> 8) & 0xFFu);

    data[3] =
      subindex;

    std::memcpy(
      &data[4],
      &value,
      static_cast<size_t>(
        bytes));

    send_frame(
      0x600u +
      static_cast<uint32_t>(
        node_id_),
      data,
      8);

    wait_sdo_ack(
      index,
      subindex);
  }

  void sdo_write_u8(
    uint16_t index,
    uint8_t subindex,
    uint8_t value)
  {
    sdo_write(
      index,
      subindex,
      value,
      1);
  }

  void sdo_write_u16(
    uint16_t index,
    uint8_t subindex,
    uint16_t value)
  {
    sdo_write(
      index,
      subindex,
      value,
      2);
  }

  void sdo_write_u32(
    uint16_t index,
    uint8_t subindex,
    uint32_t value)
  {
    sdo_write(
      index,
      subindex,
      value,
      4);
  }

  void wait_sdo_ack(
    uint16_t index,
    uint8_t subindex)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(300);

    while (
      std::chrono::steady_clock::now() <
      deadline)
    {
      struct pollfd pfd {};

      pfd.fd =
        socket_;

      pfd.events =
        POLLIN;

      const int rc =
        ::poll(
          &pfd,
          1,
          20);

      if (rc <= 0) {
        continue;
      }

      struct can_frame frame {};

      while (
        ::read(
          socket_,
          &frame,
          sizeof(frame)) > 0)
      {
        const uint32_t clean_id =
          frame.can_id &
          CAN_SFF_MASK;

        if (
          clean_id !=
            0x580u +
            static_cast<uint32_t>(
              node_id_) ||
          frame.can_dlc < 4)
        {
          continue;
        }

        const uint16_t response_index =
          static_cast<uint16_t>(
            frame.data[1]) |
          (
            static_cast<uint16_t>(
              frame.data[2])
            << 8);

        if (
          response_index != index ||
          frame.data[3] != subindex)
        {
          continue;
        }

        if (
          frame.data[0] ==
          0x60)
        {
          return;
        }

        if (
          frame.data[0] ==
            0x80 &&
          frame.can_dlc >= 8)
        {
          uint32_t abort_code = 0;

          std::memcpy(
            &abort_code,
            &frame.data[4],
            sizeof(abort_code));

          std::ostringstream oss;

          oss
            << "SDO abort 0x"
            << std::hex
            << abort_code
            << " at 0x"
            << index
            << ":"
            << std::dec
            << static_cast<int>(
              subindex);

          throw std::runtime_error(
            oss.str());
        }
      }
    }

    std::ostringstream oss;

    oss
      << "SDO timeout at 0x"
      << std::hex
      << index
      << ":"
      << std::dec
      << static_cast<int>(
        subindex);

    throw std::runtime_error(
      oss.str());
  }

  void configure_feedback_pdos()
  {
    // TPDO0:
    // 606Ch:01 left velocity
    // 606Ch:02 right velocity
    // 5 ms

    const uint32_t tpdo0_cob =
      0x180u +
      static_cast<uint32_t>(
        node_id_);

    sdo_write_u32(
      0x1800,
      0x01,
      0x80000000u |
      tpdo0_cob);

    sdo_write_u8(
      0x1A00,
      0x00,
      0);

    sdo_write_u32(
      0x1A00,
      0x01,
      0x606C0120u);

    sdo_write_u32(
      0x1A00,
      0x02,
      0x606C0220u);

    sdo_write_u8(
      0x1800,
      0x02,
      0xFF);

    sdo_write_u16(
      0x1800,
      0x03,
      0);

    sdo_write_u16(
      0x1800,
      0x05,
      10);

    sdo_write_u8(
      0x1A00,
      0x00,
      2);

    sdo_write_u32(
      0x1800,
      0x01,
      tpdo0_cob);

    // TPDO1:
    // 6077h:01 left actual current
    // 6077h:02 right actual current
    // 5 ms

    const uint32_t tpdo1_cob =
      0x280u +
      static_cast<uint32_t>(
        node_id_);

    sdo_write_u32(
      0x1801,
      0x01,
      0x80000000u |
      tpdo1_cob);

    sdo_write_u8(
      0x1A01,
      0x00,
      0);

    sdo_write_u32(
      0x1A01,
      0x01,
      0x60770110u);

    sdo_write_u32(
      0x1A01,
      0x02,
      0x60770210u);

    sdo_write_u8(
      0x1801,
      0x02,
      0xFF);

    sdo_write_u16(
      0x1801,
      0x03,
      0);

    sdo_write_u16(
      0x1801,
      0x05,
      10);

    sdo_write_u8(
      0x1A01,
      0x00,
      2);

    sdo_write_u32(
      0x1801,
      0x01,
      tpdo1_cob);

    // TPDO2:
    // 6064h:01 left position
    // 6064h:02 right position
    // 20 ms

    const uint32_t tpdo2_cob =
      0x380u +
      static_cast<uint32_t>(
        node_id_);

    sdo_write_u32(
      0x1802,
      0x01,
      0x80000000u |
      tpdo2_cob);

    sdo_write_u8(
      0x1A02,
      0x00,
      0);

    sdo_write_u32(
      0x1A02,
      0x01,
      0x60640120u);

    sdo_write_u32(
      0x1A02,
      0x02,
      0x60640220u);

    sdo_write_u8(
      0x1802,
      0x02,
      0xFF);

    sdo_write_u16(
      0x1802,
      0x03,
      0);

    sdo_write_u16(
      0x1802,
      0x05,
      40);

    sdo_write_u8(
      0x1A02,
      0x00,
      2);

    sdo_write_u32(
      0x1802,
      0x01,
      tpdo2_cob);
  }

  std::string interface_name_;
  int node_id_{1};
  int socket_{-1};
};

void log_row(
  std::ofstream & csv,
  const std::chrono::steady_clock::time_point &
    program_start,
  int trial,
  const std::string & phase,
  double phase_time_s,
  double target_current_a,
  const Feedback & fb,
  double command_start_s = std::numeric_limits<double>::quiet_NaN(),
  double command_done_s = std::numeric_limits<double>::quiet_NaN())
{
  const double host_time_s =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      program_start).count();

  csv
    << std::fixed
    << std::setprecision(9)

    << host_time_s << ','
    << trial << ','
    << phase << ','
    << phase_time_s << ','
    << target_current_a << ','
    << target_current_a << ','
    << target_current_a << ','
    << fb.left_current << ','
    << fb.right_current << ','
    << fb.left_velocity << ','
    << fb.right_velocity << ','
    << fb.left_position << ','
    << fb.right_position << ','
    << command_start_s << ','
    << command_done_s << '\n';
}

bool wait_until_stopped(
  ZlacCan & can,
  Feedback & fb,
  std::ofstream & csv,
  const std::chrono::steady_clock::time_point &
    program_start,
  int trial,
  double sample_hz,
  double threshold_rad_s,
  double hold_s,
  double timeout_s)
{
  const auto period =
    std::chrono::duration<double>(
      1.0 / sample_hz);

  can.command_current(0.0);

  const auto phase_start =
    std::chrono::steady_clock::now();

  auto next_tick =
    phase_start;

  bool below_active =
    false;

  std::chrono::steady_clock::time_point
    below_since{};

  while (g_keep_running.load()) {

    const auto now =
      std::chrono::steady_clock::now();

    const double elapsed =
      std::chrono::duration<double>(
        now -
        phase_start).count();

    can.drain_feedback(fb);

    log_row(
      csv,
      program_start,
      trial,
      "wait_stop",
      elapsed,
      0.0,
      fb);

    if (fb.velocity_valid &&
        std::chrono::duration<double>(now - fb.last_velocity_rx).count() <= 0.10) {

      const bool below =
        std::abs(
          fb.left_velocity) <=
            threshold_rad_s &&
        std::abs(
          fb.right_velocity) <=
            threshold_rad_s;

      if (below) {

        if (!below_active) {

          below_active = true;

          below_since =
            now;
        }

        const double held =
          std::chrono::duration<double>(
            now -
            below_since).count();

        if (held >= hold_s) {
          return true;
        }

      } else {
        below_active = false;
      }
    }

    if (elapsed >= timeout_s) {
      return false;
    }

    next_tick +=
      std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
          period);

    std::this_thread::sleep_until(
      next_tick);
  }

  return false;
}

enum class StepStopReason { SpeedLimit, Timeout, NoSwitch, Aborted };

bool feedback_fresh(const Feedback & fb,
  std::chrono::steady_clock::time_point now,
  double limit_s = 0.10)
{
  if (!fb.velocity_valid || !fb.current_valid) return false;
  return std::chrono::duration<double>(now - fb.last_velocity_rx).count() <= limit_s &&
         std::chrono::duration<double>(now - fb.last_current_rx).count() <= limit_s;
}

StepStopReason run_switch_step(
  ZlacCan & can, Feedback & fb, std::ofstream & csv,
  const std::chrono::steady_clock::time_point & program_start,
  int trial, const Options & opt)
{
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  auto relative = [&program_start](Clock::time_point tp) {
    return std::chrono::duration<double>(tp - program_start).count();
  };
  auto elapsed = [&start](Clock::time_point tp) {
    return std::chrono::duration<double>(tp - start).count();
  };
  auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / opt.sample_hz));
  auto next_tick = Clock::now();
  bool switched = false;
  double target = opt.initial_current_a;
  StepStopReason reason = StepStopReason::Timeout;

  can.drain_feedback(fb);
  log_row(csv, program_start, trial, "initial_command_before",
    elapsed(Clock::now()), 0.0, fb);
  auto cmd_start = Clock::now();
  can.command_current(target);  // two SDO writes and acknowledgement
  auto cmd_done = Clock::now();
  log_row(csv, program_start, trial, "initial_command_after",
    elapsed(cmd_done), target, fb,
    relative(cmd_start), relative(cmd_done));
  next_tick = Clock::now();

  while (g_keep_running.load()) {
    can.drain_feedback(fb);
    const auto current_now = Clock::now();
    const double dt = elapsed(current_now);

    if (!feedback_fresh(fb, current_now)) {
      throw std::runtime_error("TPDO velocity/current feedback stale (>100 ms)");
    }

    log_row(csv, program_start, trial,
      switched ? "post_switch" : "pre_switch", dt, target, fb);

    if (std::abs(fb.left_velocity) >= opt.speed_limit_rad_s ||
        std::abs(fb.right_velocity) >= opt.speed_limit_rad_s) {
      reason = switched ? StepStopReason::SpeedLimit : StepStopReason::NoSwitch;
      break;
    }
    if (dt >= opt.step_s) {
      reason = switched ? StepStopReason::Timeout : StepStopReason::NoSwitch;
      break;
    }

    // Both wheels must have reached switch speed.
    if (!switched &&
        fb.left_velocity >= opt.switch_speed_rad_s &&
        fb.right_velocity >= opt.switch_speed_rad_s) {
      log_row(csv, program_start, trial, "switch_before",
        elapsed(Clock::now()), target, fb);
      cmd_start = Clock::now();
      can.command_current(opt.switched_current_a);
      cmd_done = Clock::now();
      target = opt.switched_current_a;
      switched = true;
      log_row(csv, program_start, trial, "switch_after",
        elapsed(cmd_done), target, fb,
        relative(cmd_start), relative(cmd_done));
      next_tick = Clock::now();
    }

    next_tick += period;
    std::this_thread::sleep_until(next_tick);
  }

  // De-energize on ALL normal exit paths.
  cmd_start = Clock::now();
  can.command_current(0.0);
  cmd_done = Clock::now();
  can.drain_feedback(fb);
  log_row(csv, program_start, trial, "cutoff",
    elapsed(cmd_done), 0.0, fb,
    relative(cmd_start), relative(cmd_done));
  csv.flush();
  if (!g_keep_running.load()) return StepStopReason::Aborted;
  return reason;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options opt = parse_options(argc, argv);
    if (!opt.run) {
      print_usage(argv[0]);
      std::cerr << "Refusing to command motors without --run.\n";
      return 2;
    }
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::ofstream csv(opt.output);
    if (!csv) throw std::runtime_error("Cannot open output CSV: " + opt.output);
    csv << "host_time_s,trial,phase,phase_time_s,target_current_A,"
        << "left_cmd_A,right_cmd_A,left_actual_A,right_actual_A,"
        << "left_omega_rad_s,right_omega_rad_s,"
        << "left_position_rad,right_position_rad,"
        << "command_start_host_s,command_done_host_s\n";

    std::cout << "Running wheel step experiment: "
      << opt.initial_current_a << " -> " << opt.switched_current_a << " A; switch at "
      << opt.switch_speed_rad_s << " rad/s; cutoff at "
      << opt.speed_limit_rad_s << " rad/s; " << opt.repeats << " repeats.\n"
      << "Motor WILL MOVE. Wheels must be lifted. Starting in 3 s.\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (!g_keep_running.load()) return 130;

    ZlacCan can(opt.can_interface, opt.node_id);
    can.open_socket();
    can.configure_for_identification();
    Feedback fb;
    const auto program_start = std::chrono::steady_clock::now();

    if (!wait_until_stopped(can, fb, csv, program_start, 0,
          opt.sample_hz, opt.stop_threshold_rad_s,
          opt.stop_hold_s, opt.stop_timeout_s)) {
      throw std::runtime_error("Initial stop condition failed");
    }
    int completed = 0;
    for (int trial = 1; trial <= opt.repeats && g_keep_running.load(); ++trial) {
      std::cout << "Trial " << trial << "/" << opt.repeats << "\n";
      if (!wait_until_stopped(can, fb, csv, program_start, trial,
          opt.sample_hz, opt.stop_threshold_rad_s,
          opt.stop_hold_s, opt.stop_timeout_s)) {
        throw std::runtime_error("Stop condition timeout; aborting experiment");
      }
      if (!feedback_fresh(fb, std::chrono::steady_clock::now())) {
        throw std::runtime_error("Stale feedback before motor command");
      }
      const auto reason = run_switch_step(can, fb, csv, program_start, trial, opt);
      if (reason == StepStopReason::NoSwitch || reason == StepStopReason::Timeout) {
        std::cerr << "Trial timed out; ending experiment for safety.\n";
        break;
      }
      if (reason == StepStopReason::Aborted) break;
      ++completed;
      std::cout << "  current switched; speed cutoff reached.\n";
    }
    can.command_current(0.0);
    can.shutdown();
    csv.flush();
    std::cout << "Finished; completed=" << completed << " CSV=" << opt.output << "\n";
    return g_keep_running.load() ? 0 : 130;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
