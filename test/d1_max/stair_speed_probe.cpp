// Manual D1 MAX hardware probe. SDK percentages go directly to Move(); no
// bridge normalization, SetSpeed(), StandUp(), or automatic speed sweep.
// See README.md in this directory before running against a robot.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "robot_sdk/sdk_client.hpp"

namespace stair_probe {
using Clock = std::chrono::steady_clock;
using robot_sdk::CtrlSource;
using robot_sdk::MotionStatus;
constexpr int command_timeout_ms = 200;
volatile std::sig_atomic_t interrupted = 0;

void signalHandler(int) { interrupted = 1; }

struct Command {
    double forward = 0, lateral = 0, yaw = 0;
};

struct Options {
    bool execute = false, help = false;
    std::string host, port = "8082", output;
    double settle = 1.0, duration = 2.0, pause = 1.0;
    std::vector<Command> samples;
};

double number(const std::string &text) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) {
        throw std::runtime_error("Invalid finite number: " + text);
    }
    return value;
}

Command parseCommand(const std::string &text) {
    std::istringstream stream(text);
    std::string part;
    std::vector<double> values;
    while (std::getline(stream, part, ',')) values.push_back(number(part));
    if (values.size() != 3 || text.back() == ',') {
        throw std::runtime_error("--sample expects forward,lateral,yaw");
    }
    for (double value : values) {
        if (std::abs(value) > 1) throw std::runtime_error("Move inputs must be in [-1, 1]");
    }
    return {values[0], values[1], values[2]};
}

Options parse(int argc, char **argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { options.help = true; continue; }
        if (arg == "--execute") { options.execute = true; continue; }
        if (i + 1 == argc) throw std::runtime_error("Missing value for " + arg);
        const std::string value = argv[++i];
        if (arg == "--host") options.host = value;
        else if (arg == "--port") options.port = value;
        else if (arg == "--output") options.output = value;
        else if (arg == "--sample") options.samples.push_back(parseCommand(value));
        else if (arg == "--settle") options.settle = number(value);
        else if (arg == "--duration") options.duration = number(value);
        else if (arg == "--pause") options.pause = number(value);
        else throw std::runtime_error("Unknown option: " + arg);
    }
    if (options.help) return options;
    const double port = number(options.port);
    if (port < 1 || port > 65535 || port != std::floor(port)) {
        throw std::runtime_error("--port must be an integer in [1, 65535]");
    }
    options.port = std::to_string(static_cast<int>(port));
    if (options.samples.empty()) throw std::runtime_error("At least one --sample is required");
    if (options.execute && options.host.empty()) throw std::runtime_error("--execute requires --host");
    if (options.settle < 0.2 || options.settle > 10 ||
        options.duration < 0.2 || options.duration > 10 ||
        options.pause < 1 || options.pause > 10) {
        throw std::runtime_error("settle/duration: [0.2,10] s; pause: [1,10] s");
    }
    return options;
}

void help() {
    std::cout << "D1 MAX Stair speed probe (preview unless --execute is supplied)\n"
              << "  --host IP --port 8082 --execute --output FILE.csv\n"
              << "  --sample FORWARD,LATERAL,YAW   Repeat for additional stages.\n"
              << "                               SDK percentages [-1,1], NOT SI speeds.\n"
              << "  --settle 1 --duration 2 --pause 1   Seconds per stage.\n"
              << "Each sample: command during settle + measure, then zero during pause.\n"
              << "No SetSpeed or StandUp. Start standing, stationary, with an attended\n"
              << "clear area and no competing SDK/bridge/navigation controller.\n"
              << "Ctrl+C requests zero speed, releases control, and saves partial CSV.\n";
}

struct Row {
    Clock::time_point received;
    long long unix_ns;
    std::string source, phase;
    std::size_t sample;
    Command command;
    std::array<double, 3> velocity;  // Forward m/s, lateral m/s, yaw rad/s.
    uint64_t robot_timestamp_ns;
    int motion, gear, control;
};

struct Snapshot {
    Clock::time_point state_at{}, speed_at{}, mc_at{};
    std::size_t state_sequence = 0;
    robot_sdk::RobotState state{};
    std::array<double, 3> speed{}, mc{};
    bool control_lost = false, fault = false, overflow = false;
};

bool finite(const std::array<double, 3> &values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

class Feedback final : public robot_sdk::IDataCallback {
public:
    void OnRobotStateData(const robot_sdk::RobotState &data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.state = data;
        snapshot_.state_at = Clock::now();
        ++snapshot_.state_sequence;
        enqueue("robot_state", {data.speed.line, data.speed.translation, data.speed.angle}, 0);
    }
    void OnSpeedData(const robot_sdk::SpeedData &data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.speed = {data.x, data.y, data.yaw};
        snapshot_.speed_at = Clock::now();
        enqueue("speed", snapshot_.speed, 0);
    }
    void OnMcData(const robot_sdk::MotionData &data) override {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.mc = {data.v_body[0], data.v_body[1], data.omega_body[2]};
        snapshot_.mc_at = Clock::now();
        enqueue("mc_body", snapshot_.mc, data.time_stamp);
    }
    void OnControlLost(const robot_sdk::ControlLostInfo &) override {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.control_lost = true;
    }
    void OnFaultData(const robot_sdk::FaultDatas &faults) override {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &fault : faults) {
            if (fault.level == robot_sdk::FaultLevel::FatalError ||
                fault.level == robot_sdk::FaultLevel::Error) snapshot_.fault = true;
        }
    }
    Snapshot snapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }
    void context(std::size_t sample, std::string phase, Command command = {}) {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_ = sample;
        phase_ = std::move(phase);
        command_ = command;
    }
    std::vector<Row> drain() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Row> rows;
        rows.swap(rows_);
        return rows;
    }

private:
    // SDK callbacks only copy into a bounded queue; all file I/O is in main.
    void enqueue(const char *source, std::array<double, 3> velocity, uint64_t timestamp) {
        if (rows_.size() >= 4096) { snapshot_.overflow = true; return; }
        const auto unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        rows_.push_back({Clock::now(), unix_ns, source, phase_, sample_, command_, velocity,
                         timestamp, static_cast<int>(snapshot_.state.motion_status),
                         static_cast<int>(snapshot_.state.speed_level),
                         static_cast<int>(snapshot_.state.control_source)});
    }
    std::mutex mutex_;
    Snapshot snapshot_;
    std::vector<Row> rows_;
    std::size_t sample_ = 0;
    std::string phase_ = "connect";
    Command command_;
};

void check(const std::error_code &error, const char *operation) {
    if (error) throw std::runtime_error(std::string(operation) + ": " + error.message());
}

bool fresh(Clock::time_point stamp, double seconds) {
    return stamp != Clock::time_point{} && Clock::now() - stamp < std::chrono::duration<double>(seconds);
}

void healthy(const Snapshot &s, bool require_stair, bool require_control) {
    if (!fresh(s.state_at, 2.5) || !fresh(s.speed_at, 0.5) || !fresh(s.mc_at, 0.5)) {
        throw std::runtime_error("State/speed/motion feedback missing or stale");
    }
    if (!finite(s.speed) || !finite(s.mc)) throw std::runtime_error("Non-finite speed feedback");
    if (s.overflow || s.fault) throw std::runtime_error("Telemetry queue overflow or robot fault");
    if (s.state.software_emergency_status == robot_sdk::EmergencyStatus::EMERGENCY_STATUS_STOP ||
        s.state.hardware_emergency_status == robot_sdk::EmergencyStatus::EMERGENCY_STATUS_STOP) {
        throw std::runtime_error("Emergency stop active; it will not be cleared by this probe");
    }
    if (require_control && (s.control_lost || s.state.control_source != CtrlSource::CTRL_SOURCE_SDK)) {
        throw std::runtime_error("SDK control lost");
    }
    if (require_stair && s.state.motion_status != MotionStatus::MOTION_STATUS_STAIR) {
        throw std::runtime_error("Robot left Stair mode");
    }
}

bool stationary(const Snapshot &s) {
    const auto stopped = [](const auto &v) {
        return std::abs(v[0]) < 0.03 && std::abs(v[1]) < 0.03 && std::abs(v[2]) < 0.05;
    };
    return finite(s.speed) && finite(s.mc) && stopped(s.speed) && stopped(s.mc);
}

struct Stats {
    std::size_t count = 0;
    std::array<double, 3> sum{}, minimum{}, maximum{};
    void add(const std::array<double, 3> &v) {
        if (!finite(v)) return;
        if (!count) minimum = maximum = v;
        for (std::size_t i = 0; i < 3; ++i) {
            sum[i] += v[i];
            minimum[i] = std::min(minimum[i], v[i]);
            maximum[i] = std::max(maximum[i], v[i]);
        }
        ++count;
    }
};

class Recording {
public:
    explicit Recording(std::ostream &output, std::size_t samples)
        : output_(output), statistics_(samples), started_(Clock::now()) {
        output_ << "receive_s,unix_ns,sample,phase,requested_forward_pct,requested_lateral_pct,"
                   "requested_yaw_pct,source,vx_mps,vy_mps,yaw_radps,robot_timestamp_ns,"
                   "motion_status,speed_level,control_source\n";
        output_ << std::setprecision(10);
    }
    void drain(Feedback &feedback) {
        for (const auto &row : feedback.drain()) {
            const double t = std::chrono::duration<double>(row.received - started_).count();
            output_ << t << ',' << row.unix_ns << ',' << row.sample << ',' << row.phase << ','
                    << row.command.forward << ',' << row.command.lateral << ',' << row.command.yaw << ','
                    << row.source << ',' << row.velocity[0] << ',' << row.velocity[1] << ',' << row.velocity[2]
                    << ',' << row.robot_timestamp_ns << ',' << row.motion << ',' << row.gear << ','
                    << row.control << '\n';
            if (row.phase == "measure" && row.sample > 0 && row.sample <= statistics_.size()) {
                if (row.source == "speed") statistics_[row.sample - 1][0].add(row.velocity);
                if (row.source == "mc_body") statistics_[row.sample - 1][1].add(row.velocity);
            }
        }
        output_.flush();
        if (!output_) throw std::runtime_error("CSV write failed");
    }
    void summary(std::size_t sample) const {
        for (std::size_t source = 0; source < 2; ++source) {
            const auto &stats = statistics_.at(sample - 1)[source];
            std::cout << "sample=" << sample << " source=" << (source ? "mc_body" : "speed")
                      << " n=" << stats.count;
            for (std::size_t axis = 0; axis < 3 && stats.count; ++axis) {
                std::cout << ' ' << (axis == 0 ? "vx" : axis == 1 ? "vy" : "yaw_rate")
                          << " mean/min/max=" << stats.sum[axis] / stats.count << '/'
                          << stats.minimum[axis] << '/' << stats.maximum[axis];
            }
            std::cout << '\n';
            if (!stats.count) throw std::runtime_error("No measurement samples collected");
        }
    }
private:
    std::ostream &output_;
    std::vector<std::array<Stats, 2>> statistics_;
    Clock::time_point started_;
};

// This guard outlives all normal and exceptional exits from execute().
struct Session {
    robot_sdk::SDKClient &client;
    bool control_attempted = false;
    ~Session() {
        if (control_attempted && client.IsConnected()) {
            for (int i = 0; i < 3; ++i) {
                const auto error = client.Move(0, 0, 0, command_timeout_ms);
                if (error) std::cerr << "Cleanup zero-speed failed: " << error.message() << '\n';
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            const auto error = client.ReleaseControl(command_timeout_ms);
            if (error) std::cerr << "ReleaseControl failed: " << error.message() << '\n';
        }
        client.Disconnect(true);
    }
};

void active(robot_sdk::SDKClient &client) {
    if (interrupted) throw std::runtime_error("Interrupted");
    if (!client.IsConnected()) throw std::runtime_error("SDK disconnected");
}

template<class Predicate>
void waitFor(robot_sdk::SDKClient &client, Feedback &feedback, Recording &recording,
             Predicate predicate, const char *description) {
    const auto deadline = Clock::now() + std::chrono::seconds(6);
    while (Clock::now() < deadline) {
        active(client);
        recording.drain(feedback);
        const auto snapshot = feedback.snapshot();
        if (snapshot.control_lost || snapshot.fault) throw std::runtime_error("Control lost or robot fault");
        if (predicate(snapshot)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error(std::string("Timeout waiting for ") + description);
}

void phase(robot_sdk::SDKClient &client, Feedback &feedback, Recording &recording,
           std::size_t sample, const std::string &label, Command command, double seconds) {
    feedback.context(sample, label, command);
    const auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
    do {
        active(client);
        healthy(feedback.snapshot(), true, true);
        check(client.Move(static_cast<float>(command.lateral), static_cast<float>(command.forward),
                          static_cast<float>(command.yaw), command_timeout_ms), "Move");
        recording.drain(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (Clock::now() < deadline);
    active(client);
    healthy(feedback.snapshot(), true, true);
    recording.drain(feedback);
}

void execute(const Options &options, std::ostream &output) {
    auto feedback = std::make_shared<Feedback>();
    Recording recording(output, options.samples.size());
    robot_sdk::ConnectionConfig config;
    config.auto_reconnect = false;
    config.connect_timeout_ms = 3000;
    robot_sdk::SDKClient client([](const std::error_code &) {}, config);
    client.SetDataCallback(feedback);
    Session session{client};
    try {
        check(client.Connect(options.host, options.port, true), "Connect");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "SDK=" << client.Version() << " firmware=" << client.SystemVersion() << '\n';
        check(client.SetSpeedReportConfig(true, 50, command_timeout_ms), "SetSpeedReportConfig");
        check(client.SetMcConfig(true, command_timeout_ms), "SetMcConfig");
        waitFor(client, *feedback, recording, [](const Snapshot &s) {
            return fresh(s.state_at, 2.5) && fresh(s.speed_at, 0.5) && fresh(s.mc_at, 0.5);
        }, "initial telemetry");
        auto s = feedback->snapshot();
        healthy(s, false, false);
        if (!stationary(s)) throw std::runtime_error("Robot must be stationary before starting");
        if (s.state.motion_status != MotionStatus::MOTION_STATUS_WALK &&
            s.state.motion_status != MotionStatus::MOTION_STATUS_GAIT &&
            s.state.motion_status != MotionStatus::MOTION_STATUS_BALANCE_STAND &&
            s.state.motion_status != MotionStatus::MOTION_STATUS_STAIR) {
            throw std::runtime_error("Start from a standing posture; this probe does not StandUp");
        }
        const auto before_control = s.state_sequence;
        session.control_attempted = true;
        check(client.TakeControl(command_timeout_ms), "TakeControl");
        waitFor(client, *feedback, recording, [before_control](const Snapshot &state) {
            return state.state_sequence > before_control &&
                   state.state.control_source == CtrlSource::CTRL_SOURCE_SDK;
        }, "SDK control feedback");
        check(client.Move(0, 0, 0, command_timeout_ms), "Initial zero speed");
        const auto before_stair = feedback->snapshot().state_sequence;
        feedback->context(0, "enter_stair");
        check(client.Stair(command_timeout_ms), "Stair");
        waitFor(client, *feedback, recording, [before_stair](const Snapshot &state) {
            return state.state_sequence > before_stair &&
                   state.state.motion_status == MotionStatus::MOTION_STATUS_STAIR;
        }, "fresh Stair state feedback");
        phase(client, *feedback, recording, 0, "baseline", {}, options.pause);
        if (!stationary(feedback->snapshot())) throw std::runtime_error("Robot did not stop in Stair mode");
        for (std::size_t i = 0; i < options.samples.size(); ++i) {
            const auto &command = options.samples[i];
            std::cout << "Starting sample " << i + 1 << ": forward=" << command.forward
                      << " lateral=" << command.lateral << " yaw=" << command.yaw << std::endl;
            phase(client, *feedback, recording, i + 1, "settle", command, options.settle);
            phase(client, *feedback, recording, i + 1, "measure", command, options.duration);
            phase(client, *feedback, recording, i + 1, "stop", {}, options.pause);
            if (!stationary(feedback->snapshot())) throw std::runtime_error("Robot did not stop between samples");
            recording.summary(i + 1);
        }
        check(client.Move(0, 0, 0, command_timeout_ms), "Final zero speed");
        check(client.ReleaseControl(command_timeout_ms), "ReleaseControl");
        session.control_attempted = false;
    } catch (...) {
        // Stop before writing any remaining data (disk I/O may itself fail).
        if (session.control_attempted && client.IsConnected()) client.Move(0, 0, 0, command_timeout_ms);
        feedback->context(0, "aborted");
        try { recording.drain(*feedback); } catch (...) {}
        throw;
    }
}

int run(int argc, char **argv) {
    try {
        const auto options = parse(argc, argv);
        if (options.help) { help(); return 0; }
        std::cout << (options.execute ? "EXECUTE" : "PREVIEW (no SDK connection)")
                  << ": " << options.samples.size() << " samples; settle=" << options.settle
                  << "s measure=" << options.duration << "s stop=" << options.pause << "s\n";
        for (std::size_t i = 0; i < options.samples.size(); ++i) {
            const auto &s = options.samples[i];
            std::cout << i + 1 << ": Move(lateral=" << s.lateral << ", forward=" << s.forward
                      << ", yaw=" << s.yaw << ")\n";
        }
        if (!options.execute) return 0;
        std::string path = options.output;
        if (path.empty()) {
            path = "stair_speed_" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count()) + ".csv";
        }
        if (std::ifstream(path).good()) throw std::runtime_error("Output already exists: " + path);
        std::ofstream output(path);
        if (!output) throw std::runtime_error("Cannot create CSV: " + path);
        std::cout << "CSV: " << path << std::endl;
        execute(options, output);
        std::cout << "Finished; zero speed sent and control released. Robot remains in Stair mode.\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Probe stopped: " << error.what() << '\n';
        return 1;
    }
}
}  // namespace stair_probe

#ifndef D1_MAX_STAIR_PROBE_NO_MAIN
int main(int argc, char **argv) {
    std::signal(SIGINT, stair_probe::signalHandler);
    std::signal(SIGTERM, stair_probe::signalHandler);
    return stair_probe::run(argc, argv);
}
#endif
