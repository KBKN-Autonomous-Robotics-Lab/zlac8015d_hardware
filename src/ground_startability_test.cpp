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
enum class TestState {PRE_ZERO, RISE, HOLD, POST_ZERO, DONE};

struct Options
{
  std::string can_interface{"can0"};
  int node_id{1};
  WheelSelection wheel{WheelSelection::BOTH};
  Direction direction{Direction::FORWARD};

  double pre_s{1.0};
  double post_s{1.0};
  double sample_hz{200.0};

  double test_current_a{0.4};
  double rise_rate_a_per_s{5.0};
  double hold_s{3.0};
  double start_threshold_rad_s{0.10};
  double start_confirm_s{0.20};

  double driver_torque_slope_a_per_s{30.0};
  double feedback_timeout_s{0.030};
  double sdo_timeout_s{0.010};

  double stop_threshold_rad_s{0.15};
  double stop_hold_s{0.50};
  double stop_timeout_s{20.0};

  // 0 disables this guard. Enable only with a known-safe limit.
  double actual_current_abort_a{0.0};

  bool run{false};
  std::string output{"ground_startability_test.csv"};
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
  bool start_confirmed{false};
  bool target_reached{false};
  bool feedback_watchdog_tripped{false};
  bool actual_current_guard_tripped{false};
  bool sdo_failed{false};
  bool above_seen_during_rise{false};

  double target_reached_time_s{std::numeric_limits<double>::quiet_NaN()};
  double candidate_time_s{std::numeric_limits<double>::quiet_NaN()};
  double confirmed_time_s{std::numeric_limits<double>::quiet_NaN()};
  double candidate_after_target_s{std::numeric_limits<double>::quiet_NaN()};
  double confirmed_after_target_s{std::numeric_limits<double>::quiet_NaN()};
  double max_actual_left_a{0.0};
  double max_actual_right_a{0.0};
  double max_abs_omega_left{0.0};
  double max_abs_omega_right{0.0};
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
    case TestState::RISE: return "rise";
    case TestState::HOLD: return "hold";
    case TestState::POST_ZERO: return "post_zero";
    case TestState::DONE: return "done";
  }
  return "unknown";
}

void usage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " --run [options]\n\n"
    << "Ground fixed-current startability test for ZLAC8015D.\n"
    << "The robot WILL MOVE. Keep the test area clear and be ready to Ctrl-C.\n\n"
    << "Method:\n"
    << "  1) Verify the robot is stopped at 0 A.\n"
    << "  2) Raise current from 0 A to --test-current at --rise-rate.\n"
    << "  3) Once target current is reached, hold it for --hold seconds.\n"
    << "  4) START is confirmed when the selected wheel condition remains\n"
    << "     beyond --velocity-threshold continuously for --velocity-hold.\n"
    << "  5) On confirmation, immediately command 0 A. If the hold expires\n"
    << "     first, report NOT STARTED and command 0 A.\n\n"
    << "Current command: individual SDO 6071:01 / 6071:02.\n"
    << "Feedback: TPDO 0x181 velocity, TPDO 0x281 actual current.\n\n"
    << "Options:\n"
    << "  --can can0\n"
    << "  --node 1\n"
    << "  --wheel left|right|both        (default both)\n"
    << "  --direction forward|reverse    (default forward)\n"
    << "  --test-current 0.4             [A]\n"
    << "  --rise-rate 5.0                [A/s]\n"
    << "  --hold 3.0                     [s after target reached]\n"
    << "  --velocity-threshold 0.10      [rad/s]\n"
    << "  --velocity-hold 0.20           [s]\n"
    << "  --pre 1.0                      [s]\n"
    << "  --post 1.0                     [s]\n"
    << "  --sample-hz 200\n"
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
      if (i + 1 >= argc) throw std::runtime_error("Missing value after " + name);
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
    } else if (a == "--direction") {
      const std::string v = value(a);
      if (v == "forward") o.direction = Direction::FORWARD;
      else if (v == "reverse") o.direction = Direction::REVERSE;
      else throw std::runtime_error("--direction must be forward or reverse");
    } else if (a == "--test-current") o.test_current_a = std::stod(value(a));
    else if (a == "--rise-rate") o.rise_rate_a_per_s = std::stod(value(a));
    else if (a == "--hold") o.hold_s = std::stod(value(a));
    else if (a == "--velocity-threshold") o.start_threshold_rad_s = std::stod(value(a));
    else if (a == "--velocity-hold") o.start_confirm_s = std::stod(value(a));
    else if (a == "--pre") o.pre_s = std::stod(value(a));
    else if (a == "--post") o.post_s = std::stod(value(a));
    else if (a == "--sample-hz") o.sample_hz = std::stod(value(a));
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
    } else {
      throw std::runtime_error("Unknown option: " + a);
    }
  }

  if (o.node_id < 1 || o.node_id > 127 ||
      o.test_current_a <= 0.0 || o.test_current_a > 1.0 ||
      o.rise_rate_a_per_s <= 0.0 || o.rise_rate_a_per_s > 20.0 ||
      o.hold_s <= 0.0 || o.hold_s > 10.0 ||
      o.start_threshold_rad_s <= 0.0 || o.start_threshold_rad_s > 2.0 ||
      o.start_confirm_s <= 0.0 || o.start_confirm_s > o.hold_s ||
      o.pre_s < 0.0 || o.post_s < 0.0 ||
      o.sample_hz <= 0.0 || o.sample_hz > 250.0 ||
      o.driver_torque_slope_a_per_s <= 0.0 ||
      o.feedback_timeout_s <= 0.0 || o.sdo_timeout_s <= 0.0 ||
      o.stop_threshold_rad_s <= 0.0 || o.stop_hold_s <= 0.0 ||
      o.stop_timeout_s <= o.stop_hold_s || o.actual_current_abort_a < 0.0) {
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
      if (!below) { below = true; below_since = now; }
      if (std::chrono::duration<double>(now - below_since).count() >= o.stop_hold_s) return true;
    } else {
      below = false;
    }
    if (std::chrono::duration<double>(now - start).count() >= o.stop_timeout_s) return false;
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
  return false;
}

bool wheel_above_threshold(double omega, Direction direction, double threshold)
{
  return direction == Direction::FORWARD ? omega >= threshold : omega <= -threshold;
}

bool selected_condition(const Feedback & fb, const Options & o)
{
  const bool l = wheel_above_threshold(fb.omega_l, o.direction, o.start_threshold_rad_s);
  const bool r = wheel_above_threshold(fb.omega_r, o.direction, o.start_threshold_rad_s);
  if (o.wheel == WheelSelection::LEFT) return l;
  if (o.wheel == WheelSelection::RIGHT) return r;
  return l && r;
}

double signed_magnitude(double magnitude, Direction direction)
{
  return direction == Direction::FORWARD ? magnitude : -magnitude;
}

void selected_commands(WheelSelection wheel, Direction direction, double magnitude,
                       double & cmd_l, double & cmd_r)
{
  const double c = signed_magnitude(magnitude, direction);
  cmd_l = 0.0;
  cmd_r = 0.0;
  if (wheel == WheelSelection::LEFT || wheel == WheelSelection::BOTH) cmd_l = c;
  if (wheel == WheelSelection::RIGHT || wheel == WheelSelection::BOTH) cmd_r = c;
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

    csv << "host_time_s,state,state_time_s,"
        << "omega_left_rad_s,omega_right_rad_s,"
        << "current_actual_left_A,current_actual_right_A,"
        << "current_cmd_left_A,current_cmd_right_A,current_magnitude_A,"
        << "test_current_A,target_reached,time_since_target_reached_s,"
        << "selected_above_threshold,start_candidate_active,start_candidate_elapsed_s,"
        << "start_confirmed,above_seen_during_rise,"
        << "velocity_feedback_age_ms,current_feedback_age_ms,"
        << "sdo_left_ms,sdo_right_ms,sdo_total_ms,control_cycle_time_ms\n";

    std::cout << "Ground fixed-current startability test\n"
              << "wheel=" << wheel_name(o.wheel)
              << " direction=" << direction_name(o.direction)
              << " test_current=" << o.test_current_a << " A"
              << " rise_rate=" << o.rise_rate_a_per_s << " A/s"
              << " hold=" << o.hold_s << " s"
              << " threshold=" << o.start_threshold_rad_s << " rad/s"
              << " confirm=" << o.start_confirm_s << " s"
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
      throw std::runtime_error("Initial stop condition not reached; no test current was applied");
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
    auto target_reached_tp = test_start;
    auto candidate_start_tp = test_start;

    double current_mag = 0.0;
    bool candidate_active = false;

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
      if (fb.velocity_valid) velocity_age_s = std::chrono::duration<double>(now - fb.velocity_stamp).count();
      if (fb.current_valid) current_age_s = std::chrono::duration<double>(now - fb.current_stamp).count();

      if (!fb.velocity_valid || velocity_age_s > o.feedback_timeout_s) {
        result.feedback_watchdog_tripped = true;
        std::cerr << "Velocity feedback watchdog tripped at t=" << elapsed
                  << " s, age=" << velocity_age_s * 1000.0 << " ms\n";
        break;
      }

      result.max_actual_left_a = std::max(result.max_actual_left_a, std::abs(fb.current_l));
      result.max_actual_right_a = std::max(result.max_actual_right_a, std::abs(fb.current_r));
      result.max_abs_omega_left = std::max(result.max_abs_omega_left, std::abs(fb.omega_l));
      result.max_abs_omega_right = std::max(result.max_abs_omega_right, std::abs(fb.omega_r));

      if (o.actual_current_abort_a > 0.0 && fb.current_valid &&
          (std::abs(fb.current_l) > o.actual_current_abort_a ||
           std::abs(fb.current_r) > o.actual_current_abort_a)) {
        result.actual_current_guard_tripped = true;
        std::cerr << "Actual-current guard tripped at t=" << elapsed
                  << " s: left=" << fb.current_l << " A right=" << fb.current_r << " A\n";
        break;
      }

      double dt = control_cycle_time_ms / 1000.0;
      if (!std::isfinite(dt) || dt <= 0.0) dt = nominal_dt;
      dt = clamp(dt, 0.001, 0.020);

      const bool above = selected_condition(fb, o);
      double time_since_target = result.target_reached ?
        std::chrono::duration<double>(now - target_reached_tp).count() :
        std::numeric_limits<double>::quiet_NaN();
      double candidate_elapsed = candidate_active ?
        std::chrono::duration<double>(now - candidate_start_tp).count() : 0.0;

      if (state == TestState::PRE_ZERO) {
        current_mag = 0.0;
        candidate_active = false;
        if (state_time >= o.pre_s) {
          state = TestState::RISE;
          state_start = now;
        }

      } else if (state == TestState::RISE) {
        if (above) result.above_seen_during_rise = true;
        current_mag = std::min(o.test_current_a, current_mag + o.rise_rate_a_per_s * dt);
        if (current_mag >= o.test_current_a - 1e-9) {
          current_mag = o.test_current_a;
          result.target_reached = true;
          result.target_reached_time_s = elapsed;
          target_reached_tp = now;
          state = TestState::HOLD;
          state_start = now;
          candidate_active = false;
        }

      } else if (state == TestState::HOLD) {
        current_mag = o.test_current_a;
        time_since_target = std::chrono::duration<double>(now - target_reached_tp).count();

        if (above) {
          if (!candidate_active) {
            candidate_active = true;
            candidate_start_tp = now;
            result.candidate_time_s = elapsed;
            result.candidate_after_target_s = time_since_target;
          }
          candidate_elapsed = std::chrono::duration<double>(now - candidate_start_tp).count();
          if (candidate_elapsed >= o.start_confirm_s) {
            result.start_confirmed = true;
            result.confirmed_time_s = elapsed;
            result.confirmed_after_target_s = time_since_target;
            current_mag = 0.0;
            state = TestState::POST_ZERO;
            state_start = now;
            std::cout << std::fixed << std::setprecision(3)
                      << "START CONFIRMED: I=" << o.test_current_a
                      << " A, candidate_after_target=" << result.candidate_after_target_s
                      << " s, confirmed_after_target=" << result.confirmed_after_target_s << " s\n";
          }
        } else {
          candidate_active = false;
          candidate_elapsed = 0.0;
        }

        if (state == TestState::HOLD && time_since_target >= o.hold_s) {
          current_mag = 0.0;
          state = TestState::POST_ZERO;
          state_start = now;
          candidate_active = false;
          std::cout << "START NOT CONFIRMED within hold time at "
                    << o.test_current_a << " A\n";
        }

      } else if (state == TestState::POST_ZERO) {
        current_mag = 0.0;
        candidate_active = false;
        if (state_time >= o.post_s) state = TestState::DONE;
      }

      double cmd_l = 0.0, cmd_r = 0.0;
      selected_commands(o.wheel, o.direction, current_mag, cmd_l, cmd_r);

      SdoTiming sdo_timing;
      try {
        can.send_current_sdo_individual(cmd_l, cmd_r, sdo_timing);
      } catch (const std::exception & e) {
        result.sdo_failed = true;
        std::cerr << "SDO current write failed at t=" << elapsed << " s: " << e.what() << "\n";
        break;
      }

      time_since_target = result.target_reached ?
        std::chrono::duration<double>(now - target_reached_tp).count() :
        std::numeric_limits<double>::quiet_NaN();
      candidate_elapsed = candidate_active ?
        std::chrono::duration<double>(now - candidate_start_tp).count() : 0.0;

      csv << std::fixed << std::setprecision(9)
          << elapsed << ',' << state_name(state) << ','
          << std::chrono::duration<double>(now - state_start).count() << ','
          << fb.omega_l << ',' << fb.omega_r << ','
          << fb.current_l << ',' << fb.current_r << ','
          << cmd_l << ',' << cmd_r << ',' << current_mag << ','
          << o.test_current_a << ',' << (result.target_reached ? 1 : 0) << ',';
      if (std::isfinite(time_since_target)) csv << time_since_target;
      csv << ',' << (above ? 1 : 0) << ',' << (candidate_active ? 1 : 0) << ','
          << candidate_elapsed << ',' << (result.start_confirmed ? 1 : 0) << ','
          << (result.above_seen_during_rise ? 1 : 0) << ','
          << velocity_age_s * 1000.0 << ',' << current_age_s * 1000.0 << ','
          << sdo_timing.left_ms << ',' << sdo_timing.right_ms << ','
          << sdo_timing.total_ms << ',' << control_cycle_time_ms << '\n';

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
              << "Result: " << (result.start_confirmed ? "STARTED" : "NOT STARTED") << "\n"
              << "  test current       : " << o.test_current_a << " A\n";
    if (result.target_reached) {
      std::cout << "  target reached t   : " << result.target_reached_time_s << " s\n";
    }
    if (result.start_confirmed) {
      std::cout << "  start candidate    : " << result.candidate_after_target_s << " s after target\n"
                << "  start confirmed    : " << result.confirmed_after_target_s << " s after target\n";
    }
    std::cout << "  above during rise  : " << (result.above_seen_during_rise ? "yes" : "no") << "\n"
              << "  max |omega| L/R    : " << result.max_abs_omega_left << " / "
              << result.max_abs_omega_right << " rad/s\n"
              << "  max |Iactual| L/R  : " << result.max_actual_left_a << " / "
              << result.max_actual_right_a << " A\n"
              << "  CSV                : " << o.output << "\n";

    if (result.sdo_failed) return 4;
    if (result.feedback_watchdog_tripped) return 3;
    if (result.actual_current_guard_tripped) return 5;
    if (!g_run.load()) return 130;
    return result.start_confirmed ? 0 : 6;

  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
