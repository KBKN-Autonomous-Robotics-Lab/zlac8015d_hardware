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
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr double kRpm01ToRadPerSec = 0.01047197551;
constexpr double kEncoderCountsPerRev = 4096.0;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kRadPerEncoderCount = kTwoPi / kEncoderCountsPerRev;

std::atomic<bool> g_keep_running{true};

void signal_handler(int)
{
  g_keep_running.store(false);
}

struct Options
{
  std::string can_interface{"can0"};
  int node_id{1};
  std::vector<double> currents_a{1.0, 1.5, 2.0, 2.5, 3.0};
  int repeats{5};
  double pulse_s{0.30};
  double settle_s{1.00};
  double sample_hz{200.0};
  double max_current_a{3.0};
  bool bidirectional{false};
  bool run{false};
  std::string output{"b0_identification.csv"};
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

  int32_t raw_left{0};
  int32_t raw_right{0};
  int32_t last_raw_left{0};
  int32_t last_raw_right{0};
  int64_t accumulated_left{0};
  int64_t accumulated_right{0};
};

int64_t wrapped_count_delta(const int32_t current, const int32_t previous)
{
  int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);
  constexpr int64_t modulo = (int64_t{1} << 32);
  if (delta > INT32_MAX) {
    delta -= modulo;
  } else if (delta < INT32_MIN) {
    delta += modulo;
  }
  return delta;
}

std::vector<double> parse_currents(const std::string & text)
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
    throw std::runtime_error("--currents must contain at least one value");
  }
  return values;
}

void print_usage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " --run [options]\n\n"
    << "WARNING: this program commands motor current and moves the robot.\n"
    << "Lift/test safely first, then use a clear straight test lane.\n\n"
    << "Options:\n"
    << "  --can can0                 SocketCAN interface (default can0)\n"
    << "  --node 1                   CANopen node ID (default 1)\n"
    << "  --currents 1,1.5,2,2.5,3  Test current magnitudes in A\n"
    << "  --repeats 5                Repetitions per current (default 5)\n"
    << "  --pulse 0.30               Current-pulse duration in s\n"
    << "  --settle 1.00              Zero-current settling time in s\n"
    << "  --sample-hz 200            Logging/command loop frequency\n"
    << "  --max-current 3.0          Hard software safety limit in A\n"
    << "  --bidirectional            Also test negative current commands\n"
    << "  --output FILE.csv          Output CSV path\n"
    << "  --run                      Required to enable motor commands\n";
}

Options parse_options(int argc, char ** argv)
{
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value after " + name);
      }
      return argv[++i];
    };

    if (arg == "--can") {
      opt.can_interface = require_value(arg);
    } else if (arg == "--node") {
      opt.node_id = std::stoi(require_value(arg));
    } else if (arg == "--currents") {
      opt.currents_a = parse_currents(require_value(arg));
    } else if (arg == "--repeats") {
      opt.repeats = std::stoi(require_value(arg));
    } else if (arg == "--pulse") {
      opt.pulse_s = std::stod(require_value(arg));
    } else if (arg == "--settle") {
      opt.settle_s = std::stod(require_value(arg));
    } else if (arg == "--sample-hz") {
      opt.sample_hz = std::stod(require_value(arg));
    } else if (arg == "--max-current") {
      opt.max_current_a = std::stod(require_value(arg));
    } else if (arg == "--output") {
      opt.output = require_value(arg);
    } else if (arg == "--bidirectional") {
      opt.bidirectional = true;
    } else if (arg == "--run") {
      opt.run = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (opt.node_id < 1 || opt.node_id > 127 || opt.repeats <= 0 ||
      opt.pulse_s <= 0.0 || opt.settle_s < 0.0 || opt.sample_hz <= 0.0 ||
      opt.max_current_a <= 0.0)
  {
    throw std::runtime_error("Invalid numeric option");
  }

  for (double current : opt.currents_a) {
    if (current <= 0.0 || current > opt.max_current_a) {
      throw std::runtime_error(
        "Each --currents value must be > 0 and <= --max-current");
    }
  }
  return opt;
}

class ZlacCan
{
public:
  ZlacCan(std::string interface_name, int node_id)
  : interface_name_(std::move(interface_name)), node_id_(node_id)
  {
  }

  ~ZlacCan()
  {
    if (socket_ >= 0) {
      command_current(0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      try {
        sdo_write_u16(0x6040, 0x00, 0x0006);
      } catch (...) {
      }
      close(socket_);
    }
  }

  void open_socket()
  {
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
      throw std::runtime_error("Failed to create CAN socket");
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
      throw std::runtime_error("Failed to get interface index for " + interface_name_);
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("Failed to bind CAN socket to " + interface_name_);
    }
    fcntl(socket_, F_SETFL, O_NONBLOCK);
  }

  void configure_for_identification()
  {
    nmt(0x80);  // pre-operational
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    configure_feedback_pdos();
    configure_torque_rpdo();

    // Synchronized left/right command update flag.
    sdo_write_u16(0x200F, 0x00, 1);

    nmt(0x01);  // operational
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    sdo_write_u8(0x6060, 0x00, 4);  // Profile Torque Mode
    command_current(0.0);

    // CiA402 servo on: shutdown -> switch on -> enable operation.
    sdo_write_u16(0x6040, 0x00, 0x0006);
    sdo_write_u16(0x6040, 0x00, 0x0007);
    sdo_write_u16(0x6040, 0x00, 0x000F);
  }

  void command_current(const double ros_forward_current_a)
  {
    // Current command object 6071h:03 uses mA.
    // On this robot left motor sign is opposite to ROS forward direction.
    const int16_t left_ma = static_cast<int16_t>(std::lround(
      std::clamp(-ros_forward_current_a * 1000.0, -30000.0, 30000.0)));
    const int16_t right_ma = static_cast<int16_t>(std::lround(
      std::clamp(ros_forward_current_a * 1000.0, -30000.0, 30000.0)));

    const uint32_t combined =
      (static_cast<uint32_t>(static_cast<uint16_t>(right_ma)) << 16) |
      static_cast<uint16_t>(left_ma);
    send_pdo(0x300u + node_id_, combined);
  }

  void drain_feedback(Feedback & fb)
  {
    struct can_frame frame {};
    while (::read(socket_, &frame, sizeof(frame)) > 0) {
      const uint32_t tpdo0 = 0x180u + node_id_;
      const uint32_t tpdo1 = 0x280u + node_id_;
      const uint32_t tpdo2 = 0x380u + node_id_;

      if (frame.can_id == tpdo0 && frame.can_dlc >= 4) {
        uint32_t value = 0;
        std::memcpy(&value, frame.data, sizeof(value));
        // Preserve the mapping/order already verified on the current robot.
        const int16_t raw_left = static_cast<int16_t>(value & 0xFFFFu);
        const int16_t raw_right = static_cast<int16_t>((value >> 16) & 0xFFFFu);
        fb.left_velocity = -static_cast<double>(raw_left) * kRpm01ToRadPerSec;
        fb.right_velocity = static_cast<double>(raw_right) * kRpm01ToRadPerSec;
        fb.velocity_valid = true;
      } else if (frame.can_id == tpdo1 && frame.can_dlc >= 4) {
        uint32_t value = 0;
        std::memcpy(&value, frame.data, sizeof(value));
        const int16_t raw_left = static_cast<int16_t>(value & 0xFFFFu);
        const int16_t raw_right = static_cast<int16_t>((value >> 16) & 0xFFFFu);
        fb.left_current = -static_cast<double>(raw_left) * 0.1;
        fb.right_current = static_cast<double>(raw_right) * 0.1;
        fb.current_valid = true;
      } else if (frame.can_id == tpdo2 && frame.can_dlc >= 8) {
        int32_t raw_left = 0;
        int32_t raw_right = 0;
        std::memcpy(&raw_left, &frame.data[0], sizeof(raw_left));
        std::memcpy(&raw_right, &frame.data[4], sizeof(raw_right));

        if (!fb.position_valid) {
          fb.raw_left = raw_left;
          fb.raw_right = raw_right;
          fb.last_raw_left = raw_left;
          fb.last_raw_right = raw_right;
          fb.accumulated_left = 0;
          fb.accumulated_right = 0;
          fb.position_valid = true;
        } else {
          fb.accumulated_left += wrapped_count_delta(raw_left, fb.last_raw_left);
          fb.accumulated_right += wrapped_count_delta(raw_right, fb.last_raw_right);
          fb.last_raw_left = raw_left;
          fb.last_raw_right = raw_right;
          fb.left_position = -static_cast<double>(fb.accumulated_left) * kRadPerEncoderCount;
          fb.right_position = static_cast<double>(fb.accumulated_right) * kRadPerEncoderCount;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sdo_write_u16(0x6040, 0x00, 0x0006);
  }

private:
  void send_frame(uint32_t can_id, const uint8_t * data, size_t size)
  {
    struct can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = static_cast<__u8>(size);
    std::memcpy(frame.data, data, size);
    if (::write(socket_, &frame, sizeof(frame)) <= 0) {
      throw std::runtime_error("CAN write failed");
    }
  }

  void send_pdo(uint32_t can_id, uint32_t payload)
  {
    uint8_t bytes[4]{};
    std::memcpy(bytes, &payload, sizeof(payload));
    send_frame(can_id, bytes, sizeof(bytes));
  }

  void nmt(uint8_t command)
  {
    const uint8_t data[2] = {command, static_cast<uint8_t>(node_id_)};
    send_frame(0x000, data, 2);
  }

  void sdo_write(uint16_t index, uint8_t subindex, uint32_t value, int bytes)
  {
    uint8_t command = 0x23;
    if (bytes == 1) {
      command = 0x2F;
    } else if (bytes == 2) {
      command = 0x2B;
    } else if (bytes != 4) {
      throw std::runtime_error("Unsupported SDO write size");
    }

    uint8_t data[8]{};
    data[0] = command;
    data[1] = static_cast<uint8_t>(index & 0xFFu);
    data[2] = static_cast<uint8_t>((index >> 8) & 0xFFu);
    data[3] = subindex;
    std::memcpy(&data[4], &value, static_cast<size_t>(bytes));
    send_frame(0x600u + node_id_, data, 8);
    wait_sdo_ack(index, subindex);
  }

  void sdo_write_u8(uint16_t index, uint8_t subindex, uint8_t value)
  {
    sdo_write(index, subindex, value, 1);
  }

  void sdo_write_u16(uint16_t index, uint8_t subindex, uint16_t value)
  {
    sdo_write(index, subindex, value, 2);
  }

  void sdo_write_u32(uint16_t index, uint8_t subindex, uint32_t value)
  {
    sdo_write(index, subindex, value, 4);
  }

  void wait_sdo_ack(uint16_t index, uint8_t subindex)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < deadline) {
      struct pollfd pfd {};
      pfd.fd = socket_;
      pfd.events = POLLIN;
      const int rc = poll(&pfd, 1, 20);
      if (rc <= 0) {
        continue;
      }

      struct can_frame frame {};
      while (::read(socket_, &frame, sizeof(frame)) > 0) {
        if (frame.can_id != 0x580u + node_id_ || frame.can_dlc < 4) {
          continue;
        }
        const uint16_t response_index =
          static_cast<uint16_t>(frame.data[1]) |
          (static_cast<uint16_t>(frame.data[2]) << 8);
        if (response_index != index || frame.data[3] != subindex) {
          continue;
        }
        if (frame.data[0] == 0x60) {
          return;
        }
        if (frame.data[0] == 0x80 && frame.can_dlc >= 8) {
          uint32_t abort_code = 0;
          std::memcpy(&abort_code, &frame.data[4], sizeof(abort_code));
          std::ostringstream oss;
          oss << "SDO abort 0x" << std::hex << abort_code
              << " at 0x" << index << ":" << std::dec << static_cast<int>(subindex);
          throw std::runtime_error(oss.str());
        }
      }
    }
    std::ostringstream oss;
    oss << "SDO timeout at 0x" << std::hex << index
        << ":" << std::dec << static_cast<int>(subindex);
    throw std::runtime_error(oss.str());
  }

  void configure_feedback_pdos()
  {
    // TPDO0: 606Ch:03 combined actual velocity, 5 ms.
    sdo_write_u8(0x1A00, 0x00, 0);
    sdo_write_u32(0x1A00, 0x01, 0x606C0320u);
    sdo_write_u8(0x1800, 0x02, 0xFF);
    sdo_write_u16(0x1800, 0x03, 50);  // 50 * 100 us = 5 ms
    sdo_write_u16(0x1800, 0x05, 10);  // 10 * 500 us = 5 ms
    sdo_write_u8(0x1A00, 0x00, 1);

    // TPDO1: 6077h:03 combined actual current, 5 ms.
    sdo_write_u8(0x1A01, 0x00, 0);
    sdo_write_u32(0x1A01, 0x01, 0x60770320u);
    sdo_write_u8(0x1801, 0x02, 0xFF);
    sdo_write_u16(0x1801, 0x03, 50);
    sdo_write_u16(0x1801, 0x05, 10);
    sdo_write_u8(0x1A01, 0x00, 1);

    // TPDO2: 6064h:01/:02 actual positions, 20 ms.
    sdo_write_u8(0x1A02, 0x00, 0);
    sdo_write_u32(0x1A02, 0x01, 0x60640120u);
    sdo_write_u32(0x1A02, 0x02, 0x60640220u);
    sdo_write_u8(0x1802, 0x02, 0xFF);
    sdo_write_u16(0x1802, 0x03, 200);
    sdo_write_u16(0x1802, 0x05, 40);
    sdo_write_u8(0x1A02, 0x00, 2);
  }

  void configure_torque_rpdo()
  {
    // RPDO1: 6071h:03 combined target current, low16=left/high16=right [mA].
    sdo_write_u8(0x1601, 0x00, 0);
    sdo_write_u32(0x1601, 0x01, 0x60710320u);
    sdo_write_u8(0x1601, 0x00, 1);
  }

  std::string interface_name_;
  int node_id_{1};
  int socket_{-1};
};

void log_row(
  std::ofstream & csv,
  const std::chrono::steady_clock::time_point & program_start,
  int trial,
  const std::string & phase,
  double phase_time_s,
  double target_current_a,
  const Feedback & fb)
{
  const double host_time_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - program_start).count();

  csv << std::fixed << std::setprecision(9)
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
      << fb.right_position << '\n';
}

void run_phase(
  ZlacCan & can,
  Feedback & fb,
  std::ofstream & csv,
  const std::chrono::steady_clock::time_point & program_start,
  int trial,
  const std::string & phase,
  double target_current_a,
  double duration_s,
  double sample_hz)
{
  const auto period = std::chrono::duration<double>(1.0 / sample_hz);
  const auto phase_start = std::chrono::steady_clock::now();
  auto next_tick = phase_start;

  while (g_keep_running.load()) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - phase_start).count();
    if (elapsed >= duration_s) {
      break;
    }

    can.command_current(target_current_a);
    can.drain_feedback(fb);
    log_row(csv, program_start, trial, phase, elapsed, target_current_a, fb);

    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next_tick);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options opt = parse_options(argc, argv);
    if (!opt.run) {
      print_usage(argv[0]);
      std::cerr << "\nRefusing to command motors without --run.\n";
      return 2;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::vector<double> test_currents = opt.currents_a;
    if (opt.bidirectional) {
      for (double current : opt.currents_a) {
        test_currents.push_back(-current);
      }
    }

    std::ofstream csv(opt.output);
    if (!csv) {
      throw std::runtime_error("Cannot open output CSV: " + opt.output);
    }
    csv << "host_time_s,trial,phase,phase_time_s,target_current_A,"
        << "left_cmd_A,right_cmd_A,left_actual_A,right_actual_A,"
        << "left_omega_rad_s,right_omega_rad_s,left_position_rad,right_position_rad\n";

    std::cout << "b0 identification will MOVE the robot.\n"
              << "CAN=" << opt.can_interface
              << " node=" << opt.node_id
              << " sample=" << opt.sample_hz << " Hz"
              << " pulse=" << opt.pulse_s << " s"
              << " repeats=" << opt.repeats
              << " output=" << opt.output << "\n";
    std::cout << "Currents [A]:";
    for (double current : test_currents) {
      std::cout << ' ' << current;
    }
    std::cout << "\nStarting in 3 seconds. Ctrl-C aborts and commands zero current.\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ZlacCan can(opt.can_interface, opt.node_id);
    can.open_socket();
    can.configure_for_identification();

    Feedback fb;
    const auto program_start = std::chrono::steady_clock::now();
    int trial = 0;

    // Initial zero-current phase also gives the TPDOs time to arrive.
    run_phase(
      can, fb, csv, program_start, trial, "initial_settle",
      0.0, std::max(1.0, opt.settle_s), opt.sample_hz);

    if (!fb.velocity_valid || !fb.current_valid) {
      throw std::runtime_error(
        "Velocity/current TPDO feedback was not received. Check PDO mapping and CAN setup.");
    }

    for (double current : test_currents) {
      for (int repeat = 0; repeat < opt.repeats && g_keep_running.load(); ++repeat) {
        ++trial;
        std::cout << "Trial " << trial << ": " << current << " A\n";

        run_phase(
          can, fb, csv, program_start, trial, "settle",
          0.0, opt.settle_s, opt.sample_hz);

        run_phase(
          can, fb, csv, program_start, trial, "pulse",
          current, opt.pulse_s, opt.sample_hz);

        can.command_current(0.0);
        csv.flush();
      }
    }

    can.command_current(0.0);
    can.shutdown();
    csv.flush();

    std::cout << "Finished. CSV written to " << opt.output << "\n"
              << "Run analyze_b0.py on this CSV to estimate b0_L and b0_R.\n";
    return g_keep_running.load() ? 0 : 130;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
