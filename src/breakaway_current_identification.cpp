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
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

constexpr double kRpm01ToRadPerSec = 0.010471975511966;
std::atomic<bool> g_run{true};

void signal_handler(int)
{
  g_run.store(false);
}

enum class WheelSelection {LEFT, RIGHT, BOTH};
enum class Direction {FORWARD, REVERSE};
enum class TestState {PRE_ZERO, RAMP, VERIFY, MAX_HOLD, POST_ZERO, DONE};

struct Options
{
  std::string can_interface{"can0"};
  int node_id{1};
  WheelSelection wheel{WheelSelection::BOTH};
  Direction direction{Direction::FORWARD};

  double pre_s{1.0};
  double post_s{1.0};
  double sample_hz{200.0};

  double start_current_a{0.0};
  double max_current_a{0.6};
  double ramp_rate_a_per_s{0.10};
  double breakaway_threshold_rad_s{0.10};
  double breakaway_hold_s{0.20};
  double max_hold_s{0.50};

  double driver_torque_slope_a_per_s{30.0};
  double feedback_timeout_s{0.030};
  double sdo_timeout_s{0.010};

  double stop_threshold_rad_s{0.15};
  double stop_hold_s{0.50};
  double stop_timeout_s{20.0};

  // 0 disables this optional guard. Set only from a known-safe hardware limit.
  double actual_current_abort_a{0.0};

  bool run{false};
  std::string output{"breakaway_current_identification.csv"};
};

struct Feedback
{
  double omega_l{0.0};
  double omega_r{0.0};
  double current_l{0.0};
  double current_r{0.0};

  bool velocity_valid{false};
  bool current_valid{false};

  std::chrono::steady_clock::time_point velocity_stamp{};
  std::chrono::steady_clock::time_point current_stamp{};
};

struct SdoTiming
{
  double left_ms{0.0};
  double right_ms{0.0};
  double total_ms{0.0};
};

struct Result
{
  bool breakaway_confirmed{false};
  bool max_current_reached{false};
  bool feedback_watchdog_tripped{false};
  bool actual_current_guard_tripped{false};
  bool sdo_failed{false};

  double breakaway_current_a{std::numeric_limits<double>::quiet_NaN()};
  double candidate_time_s{std::numeric_limits<double>::quiet_NaN()};
  double confirmed_time_s{std::numeric_limits<double>::quiet_NaN()};
  double max_actual_left_a{0.0};
  double max_actual_right_a{0.0};
};

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

std::string wheel_name(WheelSelection w)
{
  if (w == WheelSelection::LEFT) return "left";
  if (w == WheelSelection::RIGHT) return "right";
  return "both";
}

std::string direction_name(Direction d)
{
  return d == Direction::FORWARD ? "forward" : "reverse";
}

std::string state_name(TestState s)
{
  switch (s) {
    case TestState::PRE_ZERO: return "pre_zero";
    case TestState::RAMP: return "ramp";
    case TestState::VERIFY: return "verify";
    case TestState::MAX_HOLD: return "max_hold";
    case TestState::POST_ZERO: return "post_zero";
    case TestState::DONE: return "done";
  }
  return "unknown";
}

void usage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " --run [options]\n\n"
    << "Ground-contact breakaway-current identification for ZLAC8015D.\n"
    << "Uses the same individual-SDO current commands as ladrc_step_response:\n"
    << "  left  -> 6071:01\n"
    << "  right -> 6071:02\n"
    << "Feedback:\n"
    << "  TPDO 0x181 -> 606C:01/:02 wheel velocity\n"
    << "  TPDO 0x281 -> 6077:01/:02 actual current\n\n"
    << "The robot WILL MOVE. Keep the test area clear and be ready to Ctrl-C.\n\n"
    << "Method:\n"
    << "  1) Verify both wheels are stopped.\n"
    << "  2) Ramp current from --start-current toward --max-current.\n"
    << "  3) When the selected wheel condition crosses --velocity-threshold,\n"
    << "     freeze current and verify it for --velocity-hold seconds.\n"
    << "  4) On confirmation, immediately command 0 A and log post-zero data.\n"
    << "  5) If no breakaway occurs at max current, hold max for --max-hold,\n"
    << "     then stop and report NOT REACHED.\n\n"
    << "Options:\n"
    << "  --can can0\n"
    << "  --node 1\n"
    << "  --wheel left|right|both        (default both)\n"
    << "  --direction forward|reverse    (default forward)\n"
    << "  --pre 1.0                      [s]\n"
    << "  --post 1.0                     [s]\n"
    << "  --sample-hz 200\n"
    << "  --start-current 0.0            [A]\n"
    << "  --max-current 0.6              [A]\n"
    << "  --ramp-rate 0.10               [A/s]\n"
    << "  --velocity-threshold 0.10      [rad/s]\n"
    << "  --velocity-hold 0.20           [s]\n"
    << "  --max-hold 0.50                [s]\n"
    << "  --driver-torque-slope 30       [A/s]\n"
    << "  --feedback-timeout-ms 30\n"
    << "  --sdo-timeout-ms 10\n"
    << "  --stop-threshold 0.15          [rad/s]\n"
    << "  --stop-hold 0.50               [s]\n"
    << "  --stop-timeout 20              [s]\n"
    << "  --actual-current-abort 0       [A], 0=disabled\n"
    << "  --output FILE.csv\n"
    << "  --run\n";
}

Options parse_options(int argc, char ** argv)
{
  Options o;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];

    auto value = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value after " + name);
      }
      return std::string(argv[++i]);
    };

    if (a == "--can") o.can_interface = value(a);
    else if (a == "--node") o.node_id = std::stoi(value(a));
    else if (a == "--wheel") {
      const std::string v = value(a);
      if (v == "left") o.wheel = WheelSelection::LEFT;
      else if (v == "right") o.wheel = WheelSelection::RIGHT;
      else if (v == "both") o.wheel = WheelSelection::BOTH;
      else throw std::runtime_error("--wheel must be left, right, or both");
    }
    else if (a == "--direction") {
      const std::string v = value(a);
      if (v == "forward") o.direction = Direction::FORWARD;
      else if (v == "reverse") o.direction = Direction::REVERSE;
      else throw std::runtime_error("--direction must be forward or reverse");
    }
    else if (a == "--pre") o.pre_s = std::stod(value(a));
    else if (a == "--post") o.post_s = std::stod(value(a));
    else if (a == "--sample-hz") o.sample_hz = std::stod(value(a));
    else if (a == "--start-current") o.start_current_a = std::stod(value(a));
    else if (a == "--max-current") o.max_current_a = std::stod(value(a));
    else if (a == "--ramp-rate") o.ramp_rate_a_per_s = std::stod(value(a));
    else if (a == "--velocity-threshold") o.breakaway_threshold_rad_s = std::stod(value(a));
    else if (a == "--velocity-hold") o.breakaway_hold_s = std::stod(value(a));
    else if (a == "--max-hold") o.max_hold_s = std::stod(value(a));
    else if (a == "--driver-torque-slope") o.driver_torque_slope_a_per_s = std::stod(value(a));
    else if (a == "--feedback-timeout-ms") o.feedback_timeout_s = std::stod(value(a)) / 1000.0;
    else if (a == "--sdo-timeout-ms") o.sdo_timeout_s = std::stod(value(a)) / 1000.0;
    else if (a == "--stop-threshold") o.stop_threshold_rad_s = std::stod(value(a));
    else if (a == "--stop-hold") o.stop_hold_s = std::stod(value(a));
    else if (a == "--stop-timeout") o.stop_timeout_s = std::stod(value(a));
    else if (a == "--actual-current-abort") o.actual_current_abort_a = std::stod(value(a));
    else if (a == "--output") o.output = value(a);
    else if (a == "--run") o.run = true;
    else if (a == "--help" || a == "-h") {
      usage(argv[0]);
      std::exit(0);
    }
    else {
      throw std::runtime_error("Unknown option: " + a);
    }
  }

  if (
    o.node_id < 1 || o.node_id > 127 ||
    o.pre_s < 0.0 || o.post_s < 0.0 ||
    o.sample_hz <= 0.0 || o.sample_hz > 250.0 ||
    o.start_current_a < 0.0 || o.start_current_a > 1.0 ||
    o.max_current_a <= 0.0 || o.max_current_a > 1.0 ||
    o.start_current_a >= o.max_current_a ||
    o.ramp_rate_a_per_s <= 0.0 || o.ramp_rate_a_per_s > 20.0 ||
    o.breakaway_threshold_rad_s <= 0.0 || o.breakaway_threshold_rad_s > 2.0 ||
    o.breakaway_hold_s <= 0.0 || o.breakaway_hold_s > 2.0 ||
    o.max_hold_s < 0.0 || o.max_hold_s > 5.0 ||
    o.driver_torque_slope_a_per_s <= 0.0 ||
    o.feedback_timeout_s <= 0.0 ||
    o.sdo_timeout_s <= 0.0 ||
    o.stop_threshold_rad_s <= 0.0 ||
    o.stop_hold_s <= 0.0 ||
    o.stop_timeout_s <= o.stop_hold_s ||
    o.actual_current_abort_a < 0.0)
  {
    throw std::runtime_error("Invalid or unsafe numeric option");
  }

  return o;
}

class ZlacCan
{
public:
  ZlacCan(std::string ifname, int node, double sdo_timeout_s)
  : ifname_(std::move(ifname)),
    node_(node),
    sdo_timeout_s_(sdo_timeout_s)
  {}

  ~ZlacCan()
  {
    try {
      if (fd_ >= 0) {
        SdoTiming ignored;
        send_current_sdo_individual(0.0, 0.0, ignored);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        sdo_u16(0x6040, 0x00, 0x0006);
      }
    } catch (...) {}

    if (fd_ >= 0) close(fd_);
  }

  void open_socket()
  {
    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("Failed to create CAN socket");

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      throw std::runtime_error("Failed to resolve " + ifname_);
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("Failed to bind " + ifname_);
    }

    fcntl(fd_, F_SETFL, O_NONBLOCK);
  }

  void configure(double driver_torque_slope_a_per_s)
  {
    nmt(0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    configure_feedback_pdos();

    // This device has been observed to power up with 200F:00 = 1.
    // Individual SDO torque control requires asynchronous control = 0.
    sdo_u16(0x200F, 0x00, 0);

    nmt(0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    sdo_u8(0x6060, 0x00, 4);  // Profile Torque Mode

    const uint32_t slope_ma_per_s = static_cast<uint32_t>(
      std::lround(clamp(driver_torque_slope_a_per_s * 1000.0, 1.0, 32767.0)));

    sdo_u32(0x6087, 0x01, slope_ma_per_s);
    sdo_u32(0x6087, 0x02, slope_ma_per_s);

    SdoTiming ignored;
    send_current_sdo_individual(0.0, 0.0, ignored);

    sdo_u16(0x6040, 0x00, 0x0006);
    sdo_u16(0x6040, 0x00, 0x0007);
    sdo_u16(0x6040, 0x00, 0x000F);
  }

  void drain_feedback(Feedback & fb)
  {
    while (!pending_frames_.empty()) {
      const struct can_frame frame = pending_frames_.front();
      pending_frames_.pop_front();
      process_feedback_frame(frame, fb);
    }

    struct can_frame frame {};
    while (::read(fd_, &frame, sizeof(frame)) > 0) {
      process_feedback_frame(frame, fb);
    }
  }

  void send_current_sdo_individual(
    double left_ros_a,
    double right_ros_a,
    SdoTiming & timing)
  {
    const int16_t left_ma = static_cast<int16_t>(
      std::lround(clamp(-left_ros_a * 1000.0, -30000.0, 30000.0)));
    const int16_t right_ma = static_cast<int16_t>(
      std::lround(clamp(right_ros_a * 1000.0, -30000.0, 30000.0)));

    const auto t0 = std::chrono::steady_clock::now();

    const auto tl0 = std::chrono::steady_clock::now();
    sdo_u16(0x6071, 0x01, static_cast<uint16_t>(left_ma));
    const auto tl1 = std::chrono::steady_clock::now();

    const auto tr0 = std::chrono::steady_clock::now();
    sdo_u16(0x6071, 0x02, static_cast<uint16_t>(right_ma));
    const auto tr1 = std::chrono::steady_clock::now();

    timing.left_ms = std::chrono::duration<double, std::milli>(tl1 - tl0).count();
    timing.right_ms = std::chrono::duration<double, std::milli>(tr1 - tr0).count();
    timing.total_ms = std::chrono::duration<double, std::milli>(tr1 - t0).count();
  }

  void safe_shutdown()
  {
    if (fd_ < 0) return;
    try {
      SdoTiming ignored;
      send_current_sdo_individual(0.0, 0.0, ignored);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      sdo_u16(0x6040, 0x00, 0x0006);
    } catch (...) {}
  }

private:
  void process_feedback_frame(const struct can_frame & frame, Feedback & fb)
  {
    if (frame.can_id == static_cast<uint32_t>(0x180 + node_) && frame.can_dlc >= 8) {
      int32_t l = 0;
      int32_t r = 0;
      std::memcpy(&l, &frame.data[0], 4);
      std::memcpy(&r, &frame.data[4], 4);

      // ROS convention: forward-positive on both wheels.
      fb.omega_l = -static_cast<double>(l) * kRpm01ToRadPerSec;
      fb.omega_r =  static_cast<double>(r) * kRpm01ToRadPerSec;
      fb.velocity_valid = true;
      fb.velocity_stamp = std::chrono::steady_clock::now();

    } else if (frame.can_id == static_cast<uint32_t>(0x280 + node_) && frame.can_dlc >= 4) {
      int16_t l = 0;
      int16_t r = 0;
      std::memcpy(&l, &frame.data[0], 2);
      std::memcpy(&r, &frame.data[2], 2);

      fb.current_l = -static_cast<double>(l) * 0.1;
      fb.current_r =  static_cast<double>(r) * 0.1;
      fb.current_valid = true;
      fb.current_stamp = std::chrono::steady_clock::now();
    }
  }

  void send(uint32_t id, const uint8_t * data, size_t n)
  {
    struct can_frame frame {};
    frame.can_id = id;
    frame.can_dlc = static_cast<__u8>(n);
    std::memcpy(frame.data, data, n);
    if (::write(fd_, &frame, sizeof(frame)) <= 0) {
      throw std::runtime_error("CAN write failed");
    }
  }

  void nmt(uint8_t cmd)
  {
    const uint8_t d[2] = {cmd, static_cast<uint8_t>(node_)};
    send(0x000, d, 2);
  }

  void sdo(uint16_t idx, uint8_t sub, uint32_t val, int bytes)
  {
    const uint8_t cs = bytes == 1 ? 0x2F : bytes == 2 ? 0x2B : 0x23;

    uint8_t d[8]{};
    d[0] = cs;
    d[1] = static_cast<uint8_t>(idx & 0xff);
    d[2] = static_cast<uint8_t>((idx >> 8) & 0xff);
    d[3] = sub;
    std::memcpy(&d[4], &val, bytes);

    send(0x600u + static_cast<uint32_t>(node_), d, 8);
    wait_ack(idx, sub);
  }

  void sdo_u8(uint16_t idx, uint8_t sub, uint8_t val) { sdo(idx, sub, val, 1); }
  void sdo_u16(uint16_t idx, uint8_t sub, uint16_t val) { sdo(idx, sub, val, 2); }
  void sdo_u32(uint16_t idx, uint8_t sub, uint32_t val) { sdo(idx, sub, val, 4); }

  void wait_ack(uint16_t idx, uint8_t sub)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(sdo_timeout_s_));

    while (std::chrono::steady_clock::now() < deadline) {
      struct pollfd pfd {};
      pfd.fd = fd_;
      pfd.events = POLLIN;

      const auto now = std::chrono::steady_clock::now();
      const double remaining_ms =
        std::chrono::duration<double, std::milli>(deadline - now).count();
      const int timeout_ms = std::max(1, static_cast<int>(std::ceil(remaining_ms)));

      if (poll(&pfd, 1, timeout_ms) <= 0) continue;

      struct can_frame f {};
      while (::read(fd_, &f, sizeof(f)) > 0) {
        if (f.can_id == static_cast<uint32_t>(0x580 + node_) && f.can_dlc >= 4) {
          const uint16_t ridx = static_cast<uint16_t>(f.data[1]) |
            (static_cast<uint16_t>(f.data[2]) << 8);

          if (ridx == idx && f.data[3] == sub) {
            if (f.data[0] == 0x60) return;

            if (f.data[0] == 0x80 && f.can_dlc >= 8) {
              uint32_t code = 0;
              std::memcpy(&code, &f.data[4], 4);
              std::ostringstream oss;
              oss << "SDO abort 0x" << std::hex << code
                  << " at 0x" << idx << ":" << std::dec << static_cast<int>(sub);
              throw std::runtime_error(oss.str());
            }
          }

          continue;
        }

        if (f.can_id == static_cast<uint32_t>(0x180 + node_) ||
            f.can_id == static_cast<uint32_t>(0x280 + node_)) {
          pending_frames_.push_back(f);
          if (pending_frames_.size() > 64) pending_frames_.pop_front();
        }
      }
    }

    std::ostringstream oss;
    oss << "SDO timeout at 0x" << std::hex << idx
        << ":" << std::dec << static_cast<int>(sub);
    throw std::runtime_error(oss.str());
  }

  void configure_feedback_pdos()
  {
    const uint32_t cob0 = 0x180u + static_cast<uint32_t>(node_);
    sdo_u32(0x1800, 0x01, 0x80000000u | cob0);
    sdo_u8(0x1A00, 0x00, 0);
    sdo_u32(0x1A00, 0x01, 0x606C0120u);
    sdo_u32(0x1A00, 0x02, 0x606C0220u);
    sdo_u8(0x1800, 0x02, 0xFF);
    sdo_u16(0x1800, 0x03, 0);
    sdo_u16(0x1800, 0x05, 10);
    sdo_u8(0x1A00, 0x00, 2);
    sdo_u32(0x1800, 0x01, cob0);

    const uint32_t cob1 = 0x280u + static_cast<uint32_t>(node_);
    sdo_u32(0x1801, 0x01, 0x80000000u | cob1);
    sdo_u8(0x1A01, 0x00, 0);
    sdo_u32(0x1A01, 0x01, 0x60770110u);
    sdo_u32(0x1A01, 0x02, 0x60770210u);
    sdo_u8(0x1801, 0x02, 0xFF);
    sdo_u16(0x1801, 0x03, 0);
    sdo_u16(0x1801, 0x05, 10);
    sdo_u8(0x1A01, 0x00, 2);
    sdo_u32(0x1801, 0x01, cob1);
  }

  std::string ifname_;
  int node_{1};
  int fd_{-1};
  double sdo_timeout_s_{0.010};
  std::deque<struct can_frame> pending_frames_;
};

bool wait_for_initial_stop(ZlacCan & can, Feedback & fb, const Options & o)
{
  SdoTiming ignored;
  can.send_current_sdo_individual(0.0, 0.0, ignored);

  const auto start = std::chrono::steady_clock::now();
  bool below = false;
  std::chrono::steady_clock::time_point below_since{};
  const auto period = std::chrono::duration<double>(1.0 / o.sample_hz);
  auto next = start;

  while (g_run.load()) {
    can.drain_feedback(fb);
    const auto now = std::chrono::steady_clock::now();

    if (fb.velocity_valid &&
        std::abs(fb.omega_l) <= o.stop_threshold_rad_s &&
        std::abs(fb.omega_r) <= o.stop_threshold_rad_s) {
      if (!below) {
        below = true;
        below_since = now;
      }
      if (std::chrono::duration<double>(now - below_since).count() >= o.stop_hold_s) {
        return true;
      }
    } else {
      below = false;
    }

    if (std::chrono::duration<double>(now - start).count() >= o.stop_timeout_s) {
      return false;
    }

    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }

  return false;
}

bool wheel_above_threshold(double omega, Direction direction, double threshold)
{
  if (direction == Direction::FORWARD) return omega >= threshold;
  return omega <= -threshold;
}

bool selected_condition(const Feedback & fb, const Options & o)
{
  const bool l = wheel_above_threshold(fb.omega_l, o.direction, o.breakaway_threshold_rad_s);
  const bool r = wheel_above_threshold(fb.omega_r, o.direction, o.breakaway_threshold_rad_s);

  if (o.wheel == WheelSelection::LEFT) return l;
  if (o.wheel == WheelSelection::RIGHT) return r;
  return l && r;
}

double signed_magnitude(double magnitude, Direction direction)
{
  return direction == Direction::FORWARD ? magnitude : -magnitude;
}

void selected_commands(
  WheelSelection wheel,
  Direction direction,
  double magnitude,
  double & cmd_l,
  double & cmd_r)
{
  const double signed_cmd = signed_magnitude(magnitude, direction);
  cmd_l = 0.0;
  cmd_r = 0.0;

  if (wheel == WheelSelection::LEFT || wheel == WheelSelection::BOTH) cmd_l = signed_cmd;
  if (wheel == WheelSelection::RIGHT || wheel == WheelSelection::BOTH) cmd_r = signed_cmd;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options o = parse_options(argc, argv);

    if (!o.run) {
      usage(argv[0]);
      std::cerr << "\nRefusing to command motors without --run.\n";
      return 2;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::ofstream csv(o.output);
    if (!csv) throw std::runtime_error("Cannot open output: " + o.output);

    csv
      << "host_time_s,state,state_time_s,"
      << "omega_left_rad_s,omega_right_rad_s,"
      << "current_actual_left_A,current_actual_right_A,"
      << "current_cmd_left_A,current_cmd_right_A,"
      << "current_magnitude_A,selected_above_threshold,"
      << "candidate_current_A,verify_elapsed_s,"
      << "velocity_feedback_age_ms,current_feedback_age_ms,"
      << "sdo_left_ms,sdo_right_ms,sdo_total_ms,control_cycle_time_ms\n";

    std::cout
      << "Ground breakaway-current identification\n"
      << "wheel=" << wheel_name(o.wheel)
      << " direction=" << direction_name(o.direction)
      << " start=" << o.start_current_a << " A"
      << " max=" << o.max_current_a << " A"
      << " ramp=" << o.ramp_rate_a_per_s << " A/s"
      << " threshold=" << o.breakaway_threshold_rad_s << " rad/s"
      << " hold=" << o.breakaway_hold_s << " s"
      << " sample=" << o.sample_hz << " Hz\n"
      << "current command: SDO 6071:01 + 6071:02\n"
      << "output=" << o.output << "\n"
      << "Starting in 3 seconds. Ctrl-C commands 0 A and disables operation.\n";

    std::this_thread::sleep_for(std::chrono::seconds(3));

    ZlacCan can(o.can_interface, o.node_id, o.sdo_timeout_s);
    can.open_socket();
    can.configure(o.driver_torque_slope_a_per_s);

    Feedback fb;
    if (!wait_for_initial_stop(can, fb, o)) {
      throw std::runtime_error("Initial stop condition not reached; no ramp was applied");
    }
    if (!fb.velocity_valid) throw std::runtime_error("No velocity TPDO feedback");

    Result result;
    TestState state = TestState::PRE_ZERO;

    const double nominal_dt = 1.0 / o.sample_hz;
    const auto period = std::chrono::duration<double>(nominal_dt);
    const auto test_start = std::chrono::steady_clock::now();
    auto state_start = test_start;
    auto next_tick = test_start;
    auto previous_cycle_start = test_start;
    auto verify_start = test_start;
    auto max_hold_start = test_start;

    double current_mag = 0.0;
    double candidate_current = std::numeric_limits<double>::quiet_NaN();

    while (g_run.load() && state != TestState::DONE) {
      const auto cycle_start = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(cycle_start - test_start).count();
      const double state_time = std::chrono::duration<double>(cycle_start - state_start).count();
      const double control_cycle_time_ms =
        std::chrono::duration<double, std::milli>(cycle_start - previous_cycle_start).count();
      previous_cycle_start = cycle_start;

      can.drain_feedback(fb);
      const auto now = std::chrono::steady_clock::now();

      double velocity_age_s = std::numeric_limits<double>::infinity();
      double current_age_s = std::numeric_limits<double>::infinity();
      if (fb.velocity_valid) {
        velocity_age_s = std::chrono::duration<double>(now - fb.velocity_stamp).count();
      }
      if (fb.current_valid) {
        current_age_s = std::chrono::duration<double>(now - fb.current_stamp).count();
      }

      if (!fb.velocity_valid || velocity_age_s > o.feedback_timeout_s) {
        result.feedback_watchdog_tripped = true;
        std::cerr << "Velocity feedback watchdog tripped at t=" << elapsed
                  << " s, age=" << velocity_age_s * 1000.0 << " ms\n";
        break;
      }

      result.max_actual_left_a = std::max(result.max_actual_left_a, std::abs(fb.current_l));
      result.max_actual_right_a = std::max(result.max_actual_right_a, std::abs(fb.current_r));

      if (o.actual_current_abort_a > 0.0 && fb.current_valid &&
          (std::abs(fb.current_l) > o.actual_current_abort_a ||
           std::abs(fb.current_r) > o.actual_current_abort_a)) {
        result.actual_current_guard_tripped = true;
        std::cerr << "Actual-current guard tripped at t=" << elapsed
                  << " s: left=" << fb.current_l
                  << " A right=" << fb.current_r << " A\n";
        break;
      }

      double dt = control_cycle_time_ms / 1000.0;
      if (!std::isfinite(dt) || dt <= 0.0) dt = nominal_dt;
      dt = clamp(dt, 0.001, 0.020);

      const bool above = selected_condition(fb, o);
      double verify_elapsed = 0.0;

      if (state == TestState::PRE_ZERO) {
        current_mag = 0.0;
        if (state_time >= o.pre_s) {
          state = TestState::RAMP;
          state_start = now;
          current_mag = o.start_current_a;
        }

      } else if (state == TestState::RAMP) {
        // Threshold is evaluated against feedback caused by the previously
        // issued current. Freeze that current while verifying persistence.
        if (above) {
          candidate_current = current_mag;
          result.candidate_time_s = elapsed;
          verify_start = now;
          state = TestState::VERIFY;
          state_start = now;
        } else if (current_mag >= o.max_current_a - 1e-9) {
          result.max_current_reached = true;
          max_hold_start = now;
          state = TestState::MAX_HOLD;
          state_start = now;
        } else {
          current_mag = std::min(o.max_current_a, current_mag + o.ramp_rate_a_per_s * dt);
        }

      } else if (state == TestState::VERIFY) {
        verify_elapsed = std::chrono::duration<double>(now - verify_start).count();

        if (!above) {
          // False/short threshold crossing: resume ramp from the frozen value.
          state = TestState::RAMP;
          state_start = now;
        } else if (verify_elapsed >= o.breakaway_hold_s) {
          result.breakaway_confirmed = true;
          result.breakaway_current_a = candidate_current;
          result.confirmed_time_s = elapsed;
          current_mag = 0.0;
          state = TestState::POST_ZERO;
          state_start = now;
          std::cout << std::fixed << std::setprecision(3)
                    << "BREAKAWAY CONFIRMED: I=" << result.breakaway_current_a
                    << " A, candidate_t=" << result.candidate_time_s
                    << " s, confirmed_t=" << result.confirmed_time_s << " s\n";
        }

      } else if (state == TestState::MAX_HOLD) {
        current_mag = o.max_current_a;

        if (above) {
          candidate_current = current_mag;
          result.candidate_time_s = elapsed;
          verify_start = now;
          state = TestState::VERIFY;
          state_start = now;
        } else if (std::chrono::duration<double>(now - max_hold_start).count() >= o.max_hold_s) {
          current_mag = 0.0;
          state = TestState::POST_ZERO;
          state_start = now;
          std::cout << "BREAKAWAY NOT REACHED at max current "
                    << o.max_current_a << " A\n";
        }

      } else if (state == TestState::POST_ZERO) {
        current_mag = 0.0;
        if (state_time >= o.post_s) {
          state = TestState::DONE;
        }
      }

      double cmd_l = 0.0;
      double cmd_r = 0.0;
      selected_commands(o.wheel, o.direction, current_mag, cmd_l, cmd_r);

      SdoTiming sdo_timing;
      try {
        can.send_current_sdo_individual(cmd_l, cmd_r, sdo_timing);
      } catch (const std::exception & e) {
        result.sdo_failed = true;
        std::cerr << "SDO current write failed at t=" << elapsed
                  << " s: " << e.what() << "\n";
        break;
      }

      if (state == TestState::VERIFY) {
        verify_elapsed = std::chrono::duration<double>(now - verify_start).count();
      } else {
        verify_elapsed = 0.0;
      }

      csv << std::fixed << std::setprecision(9)
          << elapsed << ','
          << state_name(state) << ','
          << std::chrono::duration<double>(now - state_start).count() << ','
          << fb.omega_l << ',' << fb.omega_r << ','
          << fb.current_l << ',' << fb.current_r << ','
          << cmd_l << ',' << cmd_r << ','
          << current_mag << ','
          << (above ? 1 : 0) << ',';

      if (std::isfinite(candidate_current)) csv << candidate_current;
      csv << ',' << verify_elapsed << ','
          << velocity_age_s * 1000.0 << ','
          << current_age_s * 1000.0 << ','
          << sdo_timing.left_ms << ','
          << sdo_timing.right_ms << ','
          << sdo_timing.total_ms << ','
          << control_cycle_time_ms << '\n';

      next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next_tick);
    }

    try {
      SdoTiming stop_timing;
      can.send_current_sdo_individual(0.0, 0.0, stop_timing);
    } catch (...) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    can.safe_shutdown();
    csv.flush();

    std::cout << std::fixed << std::setprecision(3)
              << "Result: "
              << (result.breakaway_confirmed ? "CONFIRMED" : "NOT CONFIRMED") << "\n";
    if (result.breakaway_confirmed) {
      std::cout << "  breakaway current : " << result.breakaway_current_a << " A\n"
                << "  candidate time    : " << result.candidate_time_s << " s\n"
                << "  confirmed time    : " << result.confirmed_time_s << " s\n";
    } else {
      std::cout << "  tested up to      : " << o.max_current_a << " A\n";
    }
    std::cout << "  max |Iactual| L  : " << result.max_actual_left_a << " A\n"
              << "  max |Iactual| R  : " << result.max_actual_right_a << " A\n"
              << "  CSV               : " << o.output << "\n";

    if (result.sdo_failed) return 4;
    if (result.feedback_watchdog_tripped) return 3;
    if (result.actual_current_guard_tripped) return 5;
    if (!g_run.load()) return 130;
    return result.breakaway_confirmed ? 0 : 6;

  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
