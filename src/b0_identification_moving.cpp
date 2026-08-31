#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

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
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

constexpr double RPM01_TO_RAD_S =
    0.010471975511966;

std::atomic<bool> g_running{true};

void signal_handler(int)
{
    g_running.store(false);
}

double elapsed_s(
    const std::chrono::steady_clock::time_point &a,
    const std::chrono::steady_clock::time_point &b)
{
    return std::chrono::duration<double>(
        a - b
    ).count();
}

struct Feedback
{
    double omega_left{0.0};
    double omega_right{0.0};

    double current_left{0.0};
    double current_right{0.0};

    bool velocity_valid{false};
    bool current_valid{false};
};

class CanInterface
{
public:
    explicit CanInterface(
        const std::string &interface_name)
    {
        socket_ = socket(
            PF_CAN,
            SOCK_RAW,
            CAN_RAW
        );

        if (socket_ < 0) {
            throw std::runtime_error(
                "Failed to create CAN socket"
            );
        }

        struct ifreq ifr {};
        std::strncpy(
            ifr.ifr_name,
            interface_name.c_str(),
            IFNAMSIZ - 1
        );

        if (ioctl(
            socket_,
            SIOCGIFINDEX,
            &ifr) < 0)
        {
            throw std::runtime_error(
                "Failed to get CAN interface index"
            );
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(
            socket_,
            reinterpret_cast<sockaddr *>(&addr),
            sizeof(addr)) < 0)
        {
            throw std::runtime_error(
                "Failed to bind CAN socket"
            );
        }

        fcntl(
            socket_,
            F_SETFL,
            O_NONBLOCK
        );
    }

    ~CanInterface()
    {
        if (socket_ >= 0) {
            try {
                zero();
            } catch (...) {
            }

            close(socket_);
        }
    }

    void send_frame(
        uint32_t id,
        const uint8_t *data,
        uint8_t dlc)
    {
        struct can_frame frame {};
        frame.can_id = id;
        frame.can_dlc = dlc;

        std::memcpy(
            frame.data,
            data,
            dlc
        );

        if (::write(
            socket_,
            &frame,
            sizeof(frame))
            != sizeof(frame))
        {
            throw std::runtime_error(
                "CAN write failed"
            );
        }
    }

    void command_current(
        double left_robot_a,
        double right_robot_a)
    {
        const int16_t left_ma =
            static_cast<int16_t>(
                std::lround(
                    -left_robot_a * 1000.0
                )
            );

        const int16_t right_ma =
            static_cast<int16_t>(
                std::lround(
                    +right_robot_a * 1000.0
                )
            );

        uint8_t data[4]{};

        std::memcpy(
            &data[0],
            &left_ma,
            2
        );

        std::memcpy(
            &data[2],
            &right_ma,
            2
        );

        send_frame(
            0x201,
            data,
            4
        );
    }

    void zero()
    {
        command_current(0.0,0.0);
        command_current(0.0,0.0);
        command_current(0.0,0.0);
    }

    void drain_feedback(
        Feedback &fb)
    {
        struct can_frame frame {};

        while (::read(
            socket_,
            &frame,
            sizeof(frame)) > 0)
        {
            if (
                frame.can_id == 0x181 &&
                frame.can_dlc >= 8)
            {
                int32_t raw_left = 0;
                int32_t raw_right = 0;

                std::memcpy(
                    &raw_left,
                    &frame.data[0],
                    4
                );

                std::memcpy(
                    &raw_right,
                    &frame.data[4],
                    4
                );

                fb.omega_left =
                    -static_cast<double>(
                        raw_left
                    ) * RPM01_TO_RAD_S;

                fb.omega_right =
                    +static_cast<double>(
                        raw_right
                    ) * RPM01_TO_RAD_S;

                fb.velocity_valid = true;
            }

            else if (
                frame.can_id == 0x281 &&
                frame.can_dlc >= 4)
            {
                int16_t raw_left = 0;
                int16_t raw_right = 0;

                std::memcpy(
                    &raw_left,
                    &frame.data[0],
                    2
                );

                std::memcpy(
                    &raw_right,
                    &frame.data[2],
                    2
                );

                fb.current_left =
                    -static_cast<double>(
                        raw_left
                    ) * 0.1;

                fb.current_right =
                    +static_cast<double>(
                        raw_right
                    ) * 0.1;

                fb.current_valid = true;
            }
        }
    }

private:
    int socket_{-1};
};

void log_row(
    std::ofstream &csv,
    double host_time_s,
    int pair_id,
    const std::string &phase,
    double phase_time_s,
    double cmd_left,
    double cmd_right,
    const Feedback &fb)
{
    csv
        << std::fixed
        << std::setprecision(9)

        << host_time_s << ","
        << pair_id << ","
        << phase << ","
        << phase_time_s << ","

        << cmd_left << ","
        << cmd_right << ","

        << fb.current_left << ","
        << fb.current_right << ","

        << fb.omega_left << ","
        << fb.omega_right
        << "\n";
}

void run_phase(
    CanInterface &can,
    Feedback &fb,
    std::ofstream &csv,
    const std::chrono::steady_clock::time_point &program_start,
    int pair_id,
    const std::string &phase,
    double current_a,
    double duration_s)
{
    constexpr double DT = 0.005;

    const auto phase_start =
        std::chrono::steady_clock::now();

    auto next =
        phase_start;

    while (g_running.load())
    {
        const auto now =
            std::chrono::steady_clock::now();

        const double phase_time =
            elapsed_s(
                now,
                phase_start
            );

        if (phase_time >= duration_s) {
            break;
        }

        const double host_time =
            elapsed_s(
                now,
                program_start
            );

        can.command_current(
            current_a,
            current_a
        );

        can.drain_feedback(
            fb
        );

        log_row(
            csv,
            host_time,
            pair_id,
            phase,
            phase_time,
            current_a,
            current_a,
            fb
        );

        next +=
            std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(
                        DT
                    )
                );

        std::this_thread::sleep_until(
            next
        );
    }
}

bool adaptive_launch(
    CanInterface &can,
    Feedback &fb,
    std::ofstream &csv,
    const std::chrono::steady_clock::time_point &program_start)
{
    constexpr double DT = 0.005;

    constexpr double START_CURRENT = 0.65;
    constexpr double STEP_CURRENT = 0.05;
    constexpr double STEP_TIME = 0.200;
    constexpr double MAX_CURRENT = 1.00;

    constexpr double SPEED_THRESHOLD = 0.40;
    constexpr int REQUIRED_SAMPLES = 20;

    constexpr double TIMEOUT = 3.0;

    const auto phase_start =
        std::chrono::steady_clock::now();

    auto next =
        phase_start;

    int count = 0;

    while (g_running.load())
    {
        const auto now =
            std::chrono::steady_clock::now();

        const double phase_time =
            elapsed_s(
                now,
                phase_start
            );

        if (phase_time >= TIMEOUT) {
            return false;
        }

        double current =
            START_CURRENT
            +
            STEP_CURRENT
            *
            std::floor(
                phase_time
                /
                STEP_TIME
            );

        current =
            std::min(
                current,
                MAX_CURRENT
            );

        const double host_time =
            elapsed_s(
                now,
                program_start
            );

        can.command_current(
            current,
            current
        );

        can.drain_feedback(
            fb
        );

        log_row(
            csv,
            host_time,
            0,
            "launch",
            phase_time,
            current,
            current,
            fb
        );

        if (
            fb.velocity_valid &&
            fb.omega_left >= SPEED_THRESHOLD &&
            fb.omega_right >= SPEED_THRESHOLD)
        {
            ++count;
        }
        else {
            count = 0;
        }

        if (count >= REQUIRED_SAMPLES) {
            return true;
        }

        next +=
            std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(
                        DT
                    )
                );

        std::this_thread::sleep_until(
            next
        );
    }

    return false;
}

} // namespace

int main(
    int argc,
    char **argv)
{
    try
    {
        std::signal(
            SIGINT,
            signal_handler
        );

        std::signal(
            SIGTERM,
            signal_handler
        );

        std::string output =
            "/home/ubuntu/"
            "b0_symmetric_pretest.csv";

        if (argc >= 2) {
            output = argv[1];
        }

        constexpr double I0 =
            0.40;

        constexpr double DELTA_I =
            0.10;

        constexpr double I_LOW =
            I0 - DELTA_I;

        constexpr double I_HIGH =
            I0 + DELTA_I;

        constexpr double SETTLE_I0_S =
            0.30;

        constexpr double PERTURB_S =
            0.15;

        constexpr double RECOVER_S =
            0.20;

        constexpr int PAIRS =
            3;

        std::ofstream csv(
            output
        );

        if (!csv) {
            throw std::runtime_error(
                "Cannot open CSV"
            );
        }

        csv
            << "host_time_s,"
            << "pair_id,"
            << "phase,"
            << "phase_time_s,"
            << "left_cmd_A,"
            << "right_cmd_A,"
            << "left_actual_A,"
            << "right_actual_A,"
            << "left_omega_rad_s,"
            << "right_omega_rad_s"
            << "\n";

        std::cout
            << "============================================\n"
            << "SYMMETRIC MOVING b0 IDENTIFICATION\n"
            << "============================================\n"
            << "This test WILL MOVE the robot.\n\n"

            << "I0      = "
            << I0
            << " A\n"

            << "Delta I = "
            << DELTA_I
            << " A\n"

            << "I_LOW   = "
            << I_LOW
            << " A\n"

            << "I_HIGH  = "
            << I_HIGH
            << " A\n"

            << "pairs   = "
            << PAIRS
            << "\n"

            << "output  = "
            << output
            << "\n\n"

            << "Starting in 3 seconds...\n";

        std::this_thread::sleep_for(
            std::chrono::seconds(
                3
            )
        );

        CanInterface can(
            "can0"
        );

        Feedback fb;

        const auto program_start =
            std::chrono::steady_clock::now();

        can.zero();

        run_phase(
            can,
            fb,
            csv,
            program_start,
            0,
            "zero_before",
            0.0,
            1.0
        );

        std::cout
            << "adaptive launch\n";

        if (!adaptive_launch(
            can,
            fb,
            csv,
            program_start))
        {
            can.zero();

            throw std::runtime_error(
                "Launch timeout"
            );
        }

        std::cout
            << "settle at I0\n";

        run_phase(
            can,
            fb,
            csv,
            program_start,
            0,
            "i0_settle",
            I0,
            SETTLE_I0_S
        );

        for (
            int pair = 1;
            pair <= PAIRS &&
            g_running.load();
            ++pair)
        {
            std::cout
                << "pair "
                << pair
                << "/"
                << PAIRS
                << "\n";

            /*
             * Alternate order to reduce drift/order bias.
             */
            const bool low_first =
                (pair % 2) == 1;

            if (low_first)
            {
                run_phase(
                    can,
                    fb,
                    csv,
                    program_start,
                    pair,
                    "low",
                    I_LOW,
                    PERTURB_S
                );

                run_phase(
                    can,
                    fb,
                    csv,
                    program_start,
                    pair,
                    "recover1",
                    I0,
                    RECOVER_S
                );

                run_phase(
                    can,
                    fb,
                    csv,
                    program_start,
                    pair,
                    "high",
                    I_HIGH,
                    PERTURB_S
                );
            }
            else
            {
                run_phase(
                    can,
                    fb,
                    csv,
                    program_start,
                    pair,
                    "high",
                    I_HIGH,
                    PERTURB_S
                );

                run_phase(
                    can,
                    fb,
                    csv,
                    program_start,
                    pair,
                    "recover1",
                    I0,
                    RECOVER_S
                );

                run_phase(
                    can,
                    fb,
                    csv,
                    program_start,
                    pair,
                    "low",
                    I_LOW,
                    PERTURB_S
                );
            }

            run_phase(
                can,
                fb,
                csv,
                program_start,
                pair,
                "recover2",
                I0,
                RECOVER_S
            );
        }

        can.zero();

        run_phase(
            can,
            fb,
            csv,
            program_start,
            0,
            "zero_after",
            0.0,
            1.0
        );

        can.zero();

        csv.flush();

        std::cout
            << "\n============================================\n"
            << "FINISHED\n"
            << "============================================\n"
            << "CSV: "
            << output
            << "\n";

        return
            g_running.load()
            ? 0
            : 130;
    }

    catch (
        const std::exception &e)
    {
        std::cerr
            << "ERROR: "
            << e.what()
            << "\n";

        return 1;
    }
}
