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

namespace
{

constexpr double kRpm01ToRadPerSec = 0.010471975511966;
std::atomic<bool> g_run{true};

void signal_handler(int)
{
  g_run.store(false);
}

enum class WheelSelection {LEFT, RIGHT, BOTH};

struct Options
{
  std::string can_interface{"can0"};
  int node_id{1};
  WheelSelection wheel{WheelSelection::BOTH};
  double reference_rad_s{5.0};
  double pre_s{1.0};
  double step_s{1.0};
  double post_s{1.0};
  double sample_hz{200.0};

  double b0_left{25.0};
  double b0_right{25.0};
  double wc{2.0};
  double wo{8.0};
  double current_limit_a{0.5};
  double current_rate_limit_a_per_s{5.0};
  double feedback_timeout_s{0.030};

  double stop_threshold_rad_s{0.15};
  double stop_hold_s{0.50};
  double stop_timeout_s{20.0};

  bool run{false};
  std::string output{"ladrc_step_response.csv"};
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

struct LadrcState
{
  double z1{0.0};
  double z2{0.0};
  double b0{25.0};
  double last_current_cmd{0.0};
  bool initialized{false};
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

void usage(const char * argv0)
{
  std::cout
    << "Usage: " << argv0 << " --run [options]\n\n"
    << "Standalone 200-Hz LADRC wheel-speed step-response test.\n"
    << "The robot WILL MOVE. Use only with wheels safely lifted for this test.\n\n"
    << "Options:\n"
    << "  --can can0                 SocketCAN interface\n"
    << "  --node 1                   CANopen node ID\n"
    << "  --wheel left|right|both    Wheel(s) receiving the speed step\n"
    << "  --reference 5.0            Step reference [rad/s]\n"
    << "  --pre 1.0                  Zero-reference pre-phase [s]\n"
    << "  --step 1.0                 Step duration [s]\n"
    << "  --post 1.0                 Zero-reference closed-loop braking phase [s]\n"
    << "  --sample-hz 200            LADRC/logging rate\n"
    << "  --b0-left 25               Left nominal input gain [(rad/s^2)/A]\n"
    << "  --b0-right 25              Right nominal input gain [(rad/s^2)/A]\n"
    << "  --wc 2.0                   Controller bandwidth [rad/s]\n"
    << "  --wo 8.0                   Observer bandwidth [rad/s]\n"
    << "  --current-limit 0.5        |I_cmd| hard limit [A]\n"
    << "  --current-rate-limit 5.0   |dI_cmd/dt| limit [A/s]\n"
    << "  --feedback-timeout-ms 30   Velocity watchdog [ms]\n"
    << "  --stop-threshold 0.15      Required initial |omega| [rad/s]\n"
    << "  --stop-hold 0.50           Required continuous stopped time [s]\n"
    << "  --stop-timeout 20          Abort if initial stop is not reached [s]\n"
    << "  --output FILE.csv          CSV output\n"
    << "  --run                      Required to enable current commands\n";
}

Options parse_options(int argc, char ** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto value = [&](const std::string & n) {
      if (i + 1 >= argc) throw std::runtime_error("Missing value after " + n);
      return std::string(argv[++i]);
    };
    if (a == "--can") o.can_interface = value(a);
    else if (a == "--node") o.node_id = std::stoi(value(a));
    else if (a == "--wheel") {
      const auto v = value(a);
      if (v == "left") o.wheel = WheelSelection::LEFT;
      else if (v == "right") o.wheel = WheelSelection::RIGHT;
      else if (v == "both") o.wheel = WheelSelection::BOTH;
      else throw std::runtime_error("--wheel must be left, right, or both");
    } else if (a == "--reference") o.reference_rad_s = std::stod(value(a));
    else if (a == "--pre") o.pre_s = std::stod(value(a));
    else if (a == "--step") o.step_s = std::stod(value(a));
    else if (a == "--post") o.post_s = std::stod(value(a));
    else if (a == "--sample-hz") o.sample_hz = std::stod(value(a));
    else if (a == "--b0-left") o.b0_left = std::stod(value(a));
    else if (a == "--b0-right") o.b0_right = std::stod(value(a));
    else if (a == "--wc") o.wc = std::stod(value(a));
    else if (a == "--wo") o.wo = std::stod(value(a));
    else if (a == "--current-limit") o.current_limit_a = std::stod(value(a));
    else if (a == "--current-rate-limit") o.current_rate_limit_a_per_s = std::stod(value(a));
    else if (a == "--feedback-timeout-ms") o.feedback_timeout_s = std::stod(value(a)) / 1000.0;
    else if (a == "--stop-threshold") o.stop_threshold_rad_s = std::stod(value(a));
    else if (a == "--stop-hold") o.stop_hold_s = std::stod(value(a));
    else if (a == "--stop-timeout") o.stop_timeout_s = std::stod(value(a));
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
      o.reference_rad_s <= 0.0 || o.reference_rad_s > 15.0 ||
      o.pre_s < 0.0 || o.step_s <= 0.0 || o.post_s < 0.0 ||
      o.sample_hz <= 0.0 || o.sample_hz > 250.0 ||
      o.b0_left <= 0.0 || o.b0_right <= 0.0 ||
      o.wc <= 0.0 || o.wo <= 0.0 ||
      o.current_limit_a <= 0.0 || o.current_limit_a > 1.5 ||
      o.current_rate_limit_a_per_s <= 0.0 ||
      o.feedback_timeout_s <= 0.0 ||
      o.stop_threshold_rad_s <= 0.0 ||
      o.stop_hold_s <= 0.0 || o.stop_timeout_s <= o.stop_hold_s)
  {
    throw std::runtime_error("Invalid or unsafe numeric option");
  }
  return o;
}

class ZlacCan
{
public:
  ZlacCan(std::string ifname, int node)
  : ifname_(std::move(ifname)), node_(node)
  {}

  ~ZlacCan()
  {
    try {
      if (fd_ >= 0) {
        send_current_rpdo(0.0, 0.0);
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
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("Failed to resolve " + ifname_);

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
      throw std::runtime_error("Failed to bind " + ifname_);
    fcntl(fd_, F_SETFL, O_NONBLOCK);
  }

  void configure(double current_slew_a_per_s)
  {
    nmt(0x80);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    configure_feedback_pdos();
    configure_torque_rpdo();

    // Asynchronous/immediate target-current update.
    sdo_u16(0x200F, 0x00, 0);
    nmt(0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    sdo_u8(0x6060, 0x00, 4);  // Profile Torque Mode

    const uint32_t slope = static_cast<uint32_t>(
      std::lround(clamp(current_slew_a_per_s * 1000.0, 1.0, 1000000.0)));
    sdo_u32(0x6087, 0x01, slope);
    sdo_u32(0x6087, 0x02, slope);

    send_current_rpdo(0.0, 0.0);
    sdo_u16(0x6040, 0x00, 0x0006);
    sdo_u16(0x6040, 0x00, 0x0007);
    sdo_u16(0x6040, 0x00, 0x000F);
  }

  void drain_feedback(Feedback & fb)
  {
    struct can_frame frame {};
    while (::read(fd_, &frame, sizeof(frame)) > 0) {
      if (frame.can_id == static_cast<uint32_t>(0x180 + node_) && frame.can_dlc >= 8) {
        int32_t l = 0, r = 0;
        std::memcpy(&l, &frame.data[0], 4);
        std::memcpy(&r, &frame.data[4], 4);
        // ROS-forward convention: left driver sign is inverted.
        fb.omega_l = -static_cast<double>(l) * kRpm01ToRadPerSec;
        fb.omega_r =  static_cast<double>(r) * kRpm01ToRadPerSec;
        fb.velocity_valid = true;
        fb.velocity_stamp = std::chrono::steady_clock::now();
      } else if (frame.can_id == static_cast<uint32_t>(0x280 + node_) && frame.can_dlc >= 4) {
        int16_t l = 0, r = 0;
        std::memcpy(&l, &frame.data[0], 2);
        std::memcpy(&r, &frame.data[2], 2);
        fb.current_l = -static_cast<double>(l) * 0.1;
        fb.current_r =  static_cast<double>(r) * 0.1;
        fb.current_valid = true;
        fb.current_stamp = std::chrono::steady_clock::now();
      }
    }
  }

  void send_current_rpdo(double left_ros_a, double right_ros_a)
  {
    const int16_t left_ma = static_cast<int16_t>(
      std::lround(clamp(-left_ros_a * 1000.0, -30000.0, 30000.0)));
    const int16_t right_ma = static_cast<int16_t>(
      std::lround(clamp(right_ros_a * 1000.0, -30000.0, 30000.0)));

    struct can_frame frame {};
    frame.can_id = 0x300u + static_cast<uint32_t>(node_);
    frame.can_dlc = 4;
    std::memcpy(&frame.data[0], &left_ma, 2);
    std::memcpy(&frame.data[2], &right_ma, 2);
    if (::write(fd_, &frame, sizeof(frame)) <= 0)
      throw std::runtime_error("RPDO current write failed");
  }

  void safe_shutdown()
  {
    if (fd_ < 0) return;
    send_current_rpdo(0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sdo_u16(0x6040, 0x00, 0x0006);
  }

private:
  void send(uint32_t id, const uint8_t * data, size_t n)
  {
    struct can_frame frame {};
    frame.can_id = id;
    frame.can_dlc = static_cast<__u8>(n);
    std::memcpy(frame.data, data, n);
    if (::write(fd_, &frame, sizeof(frame)) <= 0)
      throw std::runtime_error("CAN write failed");
  }

  void nmt(uint8_t cmd)
  {
    const uint8_t d[2] = {cmd, static_cast<uint8_t>(node_)};
    send(0x000, d, 2);
  }

  void sdo(uint16_t idx, uint8_t sub, uint32_t val, int bytes)
  {
    uint8_t cs = bytes == 1 ? 0x2F : (bytes == 2 ? 0x2B : 0x23);
    uint8_t d[8]{};
    d[0] = cs; d[1] = idx & 0xff; d[2] = (idx >> 8) & 0xff; d[3] = sub;
    std::memcpy(&d[4], &val, bytes);
    send(0x600u + node_, d, 8);
    wait_ack(idx, sub);
  }
  void sdo_u8(uint16_t i, uint8_t s, uint8_t v) {sdo(i,s,v,1);}
  void sdo_u16(uint16_t i, uint8_t s, uint16_t v) {sdo(i,s,v,2);}
  void sdo_u32(uint16_t i, uint8_t s, uint32_t v) {sdo(i,s,v,4);}

  void wait_ack(uint16_t idx, uint8_t sub)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    while (std::chrono::steady_clock::now() < deadline) {
      struct pollfd pfd {};
      pfd.fd = fd_; pfd.events = POLLIN;
      if (poll(&pfd, 1, 20) <= 0) continue;

      struct can_frame f {};
      while (::read(fd_, &f, sizeof(f)) > 0) {
        if (f.can_id == static_cast<uint32_t>(0x580 + node_) && f.can_dlc >= 4) {
          const uint16_t ridx = static_cast<uint16_t>(f.data[1]) |
                                (static_cast<uint16_t>(f.data[2]) << 8);
          if (ridx != idx || f.data[3] != sub) continue;
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
      }
    }
    std::ostringstream oss;
    oss << "SDO timeout at 0x" << std::hex << idx
        << ":" << std::dec << static_cast<int>(sub);
    throw std::runtime_error(oss.str());
  }

  void configure_feedback_pdos()
  {
    const uint32_t c0 = 0x180u + node_;
    sdo_u32(0x1800,0x01,0x80000000u|c0);
    sdo_u8 (0x1A00,0x00,0);
    sdo_u32(0x1A00,0x01,0x606C0120u);
    sdo_u32(0x1A00,0x02,0x606C0220u);
    sdo_u8 (0x1800,0x02,0xFF);
    sdo_u16(0x1800,0x03,0);
    sdo_u16(0x1800,0x05,10);
    sdo_u8 (0x1A00,0x00,2);
    sdo_u32(0x1800,0x01,c0);

    const uint32_t c1 = 0x280u + node_;
    sdo_u32(0x1801,0x01,0x80000000u|c1);
    sdo_u8 (0x1A01,0x00,0);
    sdo_u32(0x1A01,0x01,0x60770110u);
    sdo_u32(0x1A01,0x02,0x60770210u);
    sdo_u8 (0x1801,0x02,0xFF);
    sdo_u16(0x1801,0x03,0);
    sdo_u16(0x1801,0x05,10);
    sdo_u8 (0x1A01,0x00,2);
    sdo_u32(0x1801,0x01,c1);
  }

  void configure_torque_rpdo()
  {
    const uint32_t cob = 0x300u + node_;
    sdo_u32(0x1401,0x01,0x80000000u|cob);
    sdo_u8 (0x1601,0x00,0);
    sdo_u32(0x1601,0x01,0x60710110u);
    sdo_u32(0x1601,0x02,0x60710210u);
    sdo_u8 (0x1401,0x02,0xFF);
    sdo_u8 (0x1601,0x00,2);
    sdo_u32(0x1401,0x01,cob);
  }

  std::string ifname_;
  int node_{1};
  int fd_{-1};
};

double ladrc_step(
  LadrcState & s,
  double ref,
  double omega,
  double measured_current,
  double dt,
  double wc,
  double wo,
  double current_limit,
  double current_rate_limit)
{
  if (!s.initialized) {
    s.z1 = omega;
    s.z2 = 0.0;
    s.last_current_cmd = 0.0;
    s.initialized = true;
  }

  const double e = omega - s.z1;
  const double beta1 = 2.0 * wo;
  const double beta2 = wo * wo;

  const double z1_dot = s.z2 + s.b0 * measured_current + beta1 * e;
  const double z2_dot = beta2 * e;
  s.z1 += dt * z1_dot;
  s.z2 += dt * z2_dot;

  const double a_des = wc * (ref - s.z1);
  double cmd = (a_des - s.z2) / s.b0;
  cmd = clamp(cmd, -current_limit, current_limit);

  const double di = current_rate_limit * dt;
  cmd = clamp(cmd, s.last_current_cmd - di, s.last_current_cmd + di);
  s.last_current_cmd = cmd;
  return cmd;
}

bool wait_for_initial_stop(
  ZlacCan & can, Feedback & fb, const Options & o)
{
  can.send_current_rpdo(0.0, 0.0);
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
        std::abs(fb.omega_r) <= o.stop_threshold_rad_s)
    {
      if (!below) {
        below = true;
        below_since = now;
      }
      if (std::chrono::duration<double>(now - below_since).count() >= o.stop_hold_s)
        return true;
    } else {
      below = false;
    }

    if (std::chrono::duration<double>(now - start).count() >= o.stop_timeout_s)
      return false;

    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
  return false;
}

void reset_observer(LadrcState & s, double omega)
{
  s.z1 = omega;
  s.z2 = 0.0;
  s.last_current_cmd = 0.0;
  s.initialized = true;
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

    csv << "host_time_s,phase,phase_time_s,"
        << "omega_ref_left_rad_s,omega_ref_right_rad_s,"
        << "omega_left_rad_s,omega_right_rad_s,"
        << "current_actual_left_A,current_actual_right_A,"
        << "current_cmd_left_A,current_cmd_right_A,"
        << "z1_left_rad_s,z1_right_rad_s,"
        << "z2_left_rad_s2,z2_right_rad_s2,"
        << "velocity_feedback_age_ms,current_feedback_age_ms\n";

    std::cout << "LADRC step-response test\n"
              << "wheel=" << wheel_name(o.wheel)
              << " reference=" << o.reference_rad_s << " rad/s"
              << " b0=[" << o.b0_left << "," << o.b0_right << "]"
              << " wc=" << o.wc << " wo=" << o.wo
              << " Imax=" << o.current_limit_a << " A"
              << " dI/dt=" << o.current_rate_limit_a_per_s << " A/s"
              << " sample=" << o.sample_hz << " Hz\n"
              << "output=" << o.output << "\n"
              << "Starting in 3 seconds. Ctrl-C forces 0 A and disables operation.\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ZlacCan can(o.can_interface, o.node_id);
    can.open_socket();
    can.configure(o.current_rate_limit_a_per_s);

    Feedback fb;
    if (!wait_for_initial_stop(can, fb, o))
      throw std::runtime_error("Initial stop condition not reached; no step was applied");
    if (!fb.velocity_valid)
      throw std::runtime_error("No velocity TPDO feedback");

    LadrcState l, r;
    l.b0 = o.b0_left;
    r.b0 = o.b0_right;
    reset_observer(l, fb.omega_l);
    reset_observer(r, fb.omega_r);

    const double nominal_dt = 1.0 / o.sample_hz;
    const auto period = std::chrono::duration<double>(nominal_dt);
    const auto test_start = std::chrono::steady_clock::now();
    auto next = test_start;
    auto last_tick = test_start;

    const double total_s = o.pre_s + o.step_s + o.post_s;
    bool watchdog_tripped = false;

    while (g_run.load()) {
      const auto now0 = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now0 - test_start).count();
      if (elapsed >= total_s) break;

      can.drain_feedback(fb);
      const auto now = std::chrono::steady_clock::now();

      double velocity_age = std::numeric_limits<double>::infinity();
      double current_age = std::numeric_limits<double>::infinity();
      if (fb.velocity_valid)
        velocity_age = std::chrono::duration<double>(now - fb.velocity_stamp).count();
      if (fb.current_valid)
        current_age = std::chrono::duration<double>(now - fb.current_stamp).count();

      if (!fb.velocity_valid || velocity_age > o.feedback_timeout_s) {
        can.send_current_rpdo(0.0, 0.0);
        watchdog_tripped = true;
        std::cerr << "Velocity feedback watchdog tripped at t=" << elapsed
                  << " s, age=" << velocity_age * 1000.0 << " ms\n";
        break;
      }

      double dt = std::chrono::duration<double>(now - last_tick).count();
      last_tick = now;
      if (!std::isfinite(dt) || dt <= 0.0) dt = nominal_dt;
      dt = clamp(dt, 0.001, 0.020);

      std::string phase;
      double phase_time = 0.0;
      double ref_l = 0.0, ref_r = 0.0;

      if (elapsed < o.pre_s) {
        phase = "pre_zero";
        phase_time = elapsed;
      } else if (elapsed < o.pre_s + o.step_s) {
        phase = "step";
        phase_time = elapsed - o.pre_s;
        if (o.wheel == WheelSelection::LEFT || o.wheel == WheelSelection::BOTH)
          ref_l = o.reference_rad_s;
        if (o.wheel == WheelSelection::RIGHT || o.wheel == WheelSelection::BOTH)
          ref_r = o.reference_rad_s;
      } else {
        phase = "post_zero";
        phase_time = elapsed - o.pre_s - o.step_s;
      }

      const double i_meas_l = fb.current_valid ? fb.current_l : l.last_current_cmd;
      const double i_meas_r = fb.current_valid ? fb.current_r : r.last_current_cmd;

      double cmd_l = ladrc_step(
        l, ref_l, fb.omega_l, i_meas_l, dt,
        o.wc, o.wo, o.current_limit_a, o.current_rate_limit_a_per_s);
      double cmd_r = ladrc_step(
        r, ref_r, fb.omega_r, i_meas_r, dt,
        o.wc, o.wo, o.current_limit_a, o.current_rate_limit_a_per_s);

      // Non-selected wheel remains at zero current, rather than being actively
      // speed-regulated, to make single-wheel sign validation unambiguous.
      if (o.wheel == WheelSelection::LEFT) {
        cmd_r = 0.0;
        r.last_current_cmd = 0.0;
      } else if (o.wheel == WheelSelection::RIGHT) {
        cmd_l = 0.0;
        l.last_current_cmd = 0.0;
      }

      can.send_current_rpdo(cmd_l, cmd_r);

      csv << std::fixed << std::setprecision(9)
          << elapsed << ',' << phase << ',' << phase_time << ','
          << ref_l << ',' << ref_r << ','
          << fb.omega_l << ',' << fb.omega_r << ','
          << fb.current_l << ',' << fb.current_r << ','
          << cmd_l << ',' << cmd_r << ','
          << l.z1 << ',' << r.z1 << ','
          << l.z2 << ',' << r.z2 << ','
          << velocity_age * 1000.0 << ',' << current_age * 1000.0 << '\n';

      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }

    can.send_current_rpdo(0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    can.safe_shutdown();
    csv.flush();

    if (watchdog_tripped)
      return 3;

    std::cout << "Finished. CSV written to " << o.output << "\n";
    return g_run.load() ? 0 : 130;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}

