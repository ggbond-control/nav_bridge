#include "nav_bridge/r750/r750_backend.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace nav_bridge {
namespace {
using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

// TARKBOT X-Protocol (tarkbot_robot.cpp): 0xAA 0x55 | len | id | payload | checksum,
// where len counts the TOTAL frame bytes and the checksum is the uint8 sum of
// every preceding byte.
constexpr uint8_t kSyncHigh = 0xAA;
constexpr uint8_t kSyncLow = 0x55;
constexpr uint8_t kVelocityFrameId = 0x50;   // ROS -> MCU twist command
constexpr uint8_t kTelemetryFrameId = 0x10;  // MCU -> ROS 50 Hz feedback
constexpr size_t kVelocityFrameSize = 11;    // header(2) len id payload(6) checksum
constexpr size_t kTelemetryFrameSize = 25;   // header(2) len id payload(20) checksum
constexpr size_t kMaxFrameSize = 64;         // parser receive buffer bound

speed_t baudConstant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default: return 0;
    }
}

int16_t decodeInt16(uint8_t high, uint8_t low) {
    return static_cast<int16_t>(static_cast<uint16_t>(high) << 8 | low);
}

uint8_t byteSum(const uint8_t *data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) sum = static_cast<uint8_t>(sum + data[i]);
    return sum;
}

// vx/vyaw are scaled by 1000 into mm/s and mrad/s; saturation guards the int16
// fields regardless of how large the configured speed limits are.
std::array<uint8_t, kVelocityFrameSize> encodeVelocityFrame(double linear, double angular) {
    std::array<uint8_t, kVelocityFrameSize> frame{};
    frame[0] = kSyncHigh;
    frame[1] = kSyncLow;
    frame[2] = static_cast<uint8_t>(kVelocityFrameSize);
    frame[3] = kVelocityFrameId;
    const auto linear_raw = static_cast<int16_t>(
        std::llround(std::clamp(linear * 1000.0, -32767.0, 32767.0)));
    const auto angular_raw = static_cast<int16_t>(
        std::llround(std::clamp(angular * 1000.0, -32767.0, 32767.0)));
    const auto linear_bits = static_cast<uint16_t>(linear_raw);
    const auto angular_bits = static_cast<uint16_t>(angular_raw);
    frame[4] = static_cast<uint8_t>(linear_bits >> 8);
    frame[5] = static_cast<uint8_t>(linear_bits);
    frame[6] = 0;  // differential chassis: the lateral axis is always zero
    frame[7] = 0;
    frame[8] = static_cast<uint8_t>(angular_bits >> 8);
    frame[9] = static_cast<uint8_t>(angular_bits);
    frame[10] = byteSum(frame.data(), frame.size() - 1);
    return frame;
}

std::string escapeJson(const std::string &text) {
    std::string escaped = "\"";
    char buffer[8];
    for (const char character : text) {
        switch (character) {
            case '"': escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\b': escaped += "\\b"; break;
            case '\f': escaped += "\\f"; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default:
                if (static_cast<unsigned char>(character) < 0x20) {
                    std::snprintf(buffer, sizeof buffer, "\\u%04x", static_cast<unsigned char>(character));
                    escaped += buffer;
                } else {
                    escaped += character;
                }
        }
    }
    return escaped + "\"";
}

std::string jsonNumber(double value) {
    if (!std::isfinite(value)) return "null";
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.6g", value);
    return buffer;
}

void validate(const R750Options &o) {
    if (o.serial_port.empty() || o.serial_port.find_first_of("?#@ \t\r\n") != std::string::npos) {
        throw std::invalid_argument("R750 requires a serial port device path");
    }
    if (baudConstant(o.serial_baud) == 0) {
        throw std::invalid_argument("R750 serial_baud is not a standard termios rate");
    }
    if (!std::isfinite(o.cmd_vel_rate_hz) || o.cmd_vel_rate_hz < 3.0 ||
        o.cmd_vel_rate_hz > 100.0) {
        throw std::invalid_argument("R750 cmd_vel_rate_hz must be in 3..100 (protocol minimum 3 Hz)");
    }
    if (!std::isfinite(o.state_publish_rate_hz) || o.state_publish_rate_hz <= 0.0 ||
        o.state_publish_rate_hz > 100.0) {
        throw std::invalid_argument("R750 state_publish_rate_hz must be in (0, 100]");
    }
    if (!std::isfinite(o.max_linear_speed) || o.max_linear_speed <= 0.0 ||
        !std::isfinite(o.max_angular_speed) || o.max_angular_speed <= 0.0) {
        throw std::invalid_argument("R750 speed limits must be positive (m/s, rad/s)");
    }
    if (o.connect_timeout_ms <= 0 || o.reconnect_interval_ms <= 0 || o.cmd_vel_timeout_ms <= 0 ||
        o.telemetry_timeout_ms <= 0) {
        throw std::invalid_argument("R750 timeouts must be positive");
    }
    if (!std::isfinite(o.battery_empty_voltage) || !std::isfinite(o.battery_full_voltage) ||
        o.battery_empty_voltage <= 0.0 || o.battery_full_voltage <= o.battery_empty_voltage) {
        throw std::invalid_argument("R750 battery voltages must satisfy 0 < empty < full");
    }
}
}  // namespace

struct R750Backend::Impl {
    explicit Impl(R750Options value) : options(std::move(value)) {
        validate(options);
        want_control = options.startup_acquire_control;
    }

    R750Options options;
    mutable std::mutex mutex;  // guards the cached state, statistics and command cache
    std::mutex io;             // serializes port open/write against service actions and teardown
    std::mutex wait_mutex;
    std::condition_variable wake;
    std::atomic<bool> running{false};
    std::atomic<int> port_fd{-1};
    std::atomic<uint64_t> reopen_ticket{0};  // bumped to abandon the descriptor
    std::thread reader_thread, control_thread, monitor_thread;
    BackendState cached;
    bool want_control{false};
    bool local_estop{false};
    double vx{0.0}, wz{0.0};
    Clock::time_point command_time{}, telemetry_time{}, port_open_time{};
    std::string last_error, last_move_error;
    uint64_t move_failures{0}, rx_frames{0}, rx_errors{0}, tx_frames{0};
    double measured_vx{0.0}, measured_vy{0.0}, measured_vyaw{0.0};
    double battery_voltage{0.0}, last_move_interval_ms{0.0};
    std::deque<Clock::time_point> move_starts;
    StateCallback state_callback;
    FaultCallback fault_callback;
    RawVelocityCallback raw_velocity_callback;
    // Reader-thread receive state machine (tarkbot recvCallback shape).
    uint8_t rx_buffer[kMaxFrameSize]{};
    size_t rx_count{0};
    uint8_t rx_checksum{0};

    void notifyState() {
        StateCallback callback;
        BackendState snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex);
            snapshot = cached;
            callback = state_callback;
        }
        if (callback) callback(snapshot);
    }

    void reportError(const std::string &message) {
        FaultCallback callback;
        {
            std::lock_guard<std::mutex> lock(mutex);
            last_error = message;
            callback = fault_callback;
        }
        if (callback) callback({1, 2, message});
    }

    void clearCommandLocked() {
        vx = wz = 0.0;
        cached.vx = cached.vy = cached.vyaw = 0.0;
        command_time = Clock::time_point{};
    }

    BackendResult writeAll(int fd, const uint8_t *data, size_t length) const {
        size_t sent = 0;
        const auto deadline = Clock::now() + Ms(100);
        while (sent < length) {
            const ssize_t wrote = ::write(fd, data + sent, length - sent);
            if (wrote > 0) {
                sent += static_cast<size_t>(wrote);
                continue;
            }
            if (wrote < 0 && errno == EINTR) continue;
            if (wrote < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                struct pollfd descriptor;
                descriptor.fd = fd;
                descriptor.events = POLLOUT;
                descriptor.revents = 0;
                const int ready = ::poll(&descriptor, 1, 20);
                if (ready < 0 && errno != EINTR) break;
                if (Clock::now() >= deadline) break;
                continue;
            }
            break;  // hard error (or a zero-length write): treat the link as broken
        }
        if (sent == length) return {true, "R750 frame written"};
        return {false, std::string("R750 serial write failed: ") + std::strerror(errno)};
    }

    // Called with io held. The wire has no acknowledgement; success only means
    // the zero-velocity frame left this host.
    BackendResult sendZeroOnce() {
        const int fd = port_fd.load();
        if (fd < 0) return {false, "R750 serial port is closed"};
        const auto frame = encodeVelocityFrame(0.0, 0.0);
        const BackendResult result = writeAll(fd, frame.data(), frame.size());
        std::lock_guard<std::mutex> lock(mutex);
        if (result.success) {
            ++tx_frames;
        } else {
            ++move_failures;
            last_move_error = result.message;
        }
        return result;
    }

    // Called with io held. Publishes the descriptor only after it is fully
    // configured so the reader never adopts a half-initialized port.
    BackendResult openPort() {
        const int fd = ::open(options.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            const std::string message =
                "R750 open " + options.serial_port + " failed: " + std::strerror(errno);
            reportError(message);
            return {false, message};
        }
        struct termios tios {};
        if (::tcgetattr(fd, &tios) != 0) {
            const std::string message =
                "R750 tcgetattr " + options.serial_port + " failed: " + std::strerror(errno);
            ::close(fd);
            reportError(message);
            return {false, message};
        }
        cfmakeraw(&tios);  // 8N1 raw bytes: no echo, no translation, no flow control
        const speed_t rate = baudConstant(options.serial_baud);
        if (::cfsetispeed(&tios, rate) != 0 || ::cfsetospeed(&tios, rate) != 0 ||
            ::tcsetattr(fd, TCSANOW, &tios) != 0) {
            const std::string message =
                "R750 tcsetattr " + options.serial_port + " failed: " + std::strerror(errno);
            ::close(fd);
            reportError(message);
            return {false, message};
        }
        ::tcflush(fd, TCIOFLUSH);
        {
            std::lock_guard<std::mutex> lock(mutex);
            port_open_time = Clock::now();
        }
        port_fd.store(fd);
        return {true, "R750 serial port " + options.serial_port + " opened at " +
                          std::to_string(options.serial_baud) + " baud; waiting for telemetry"};
    }

    void linkDown(const std::string &message) {
        bool established = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            established = cached.connected;
            cached.connected = false;
            cached.control_owned = false;
            cached.battery_percent = -1.0;
            cached.motion_state = local_estop ? BackendMotionState::ESTOP : BackendMotionState::UNKNOWN;
            // Reconnection never re-arms motion output; only ready/stand does.
            if (established) want_control = false;
            clearCommandLocked();
            telemetry_time = Clock::time_point{};
        }
        ++reopen_ticket;  // the reader thread closes the descriptor
        reportError(message);
        notifyState();
    }

    void countRxError(const std::string &message) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++rx_errors;
        }
        reportError("R750 RX " + message);
    }

    void handleFrame(const uint8_t *buffer) {
        if (buffer[3] != kTelemetryFrameId) return;  // only 0x10 is defined so far
        std::string frame_error;
        bool first_telemetry = false;
        RawVelocityCallback callback;
        double linear = 0.0, lateral = 0.0, angular = 0.0;
        {
            std::lock_guard<std::mutex> lock(mutex);
            ++rx_frames;
            if (buffer[2] != kTelemetryFrameSize) {
                frame_error = "R750 telemetry frame has unexpected length " + std::to_string(buffer[2]);
                ++rx_errors;
            } else {
                measured_vx = decodeInt16(buffer[16], buffer[17]) / 1000.0;
                measured_vy = decodeInt16(buffer[18], buffer[19]) / 1000.0;
                measured_vyaw = decodeInt16(buffer[20], buffer[21]) / 1000.0;
                const auto centivolts =
                    static_cast<uint16_t>(static_cast<uint16_t>(buffer[22]) << 8 | buffer[23]);
                battery_voltage = centivolts / 100.0;
                cached.battery_percent = std::clamp(
                    (battery_voltage - options.battery_empty_voltage) /
                        (options.battery_full_voltage - options.battery_empty_voltage) * 100.0,
                    0.0, 100.0);
                telemetry_time = Clock::now();
                first_telemetry = !cached.connected;
                cached.connected = true;
                if (first_telemetry) {
                    cached.control_owned = options.enable_motion_output && want_control && !local_estop;
                    last_error.clear();
                }
                // The protocol exposes no motion state; classify from the last
                // sent command until the TX worker refreshes the classification.
                if (cached.motion_state == BackendMotionState::UNKNOWN) {
                    cached.motion_state = cached.vx == 0.0 && cached.vyaw == 0.0
                                              ? BackendMotionState::STANDING
                                              : BackendMotionState::MOVING;
                }
                linear = measured_vx;
                lateral = measured_vy;
                angular = measured_vyaw;
                callback = raw_velocity_callback;
            }
        }
        if (!frame_error.empty()) reportError(frame_error);
        if (first_telemetry) notifyState();
        if (callback) callback(linear, lateral, angular);
    }

    // Byte-wise X-Protocol state machine following the driver's recvCallback,
    // with length validation and error accounting the driver omits.
    void feed(const uint8_t *data, size_t length) {
        for (size_t i = 0; i < length; ++i) {
            const uint8_t byte = data[i];
            if (rx_count < 3) {  // sync on 0xAA 0x55, then the total length byte
                if (rx_count == 0) {
                    if (byte == kSyncHigh) {
                        rx_buffer[0] = byte;
                        rx_count = 1;
                    }
                } else if (rx_count == 1) {
                    if (byte == kSyncLow) {
                        rx_buffer[1] = byte;
                        rx_count = 2;
                    } else {
                        rx_count = 0;
                    }
                } else {
                    rx_buffer[2] = byte;
                    if (byte < 5 || byte > kMaxFrameSize) {
                        rx_count = 0;
                        countRxError("frame length out of bounds");
                    } else {
                        rx_count = 3;
                        rx_checksum = static_cast<uint8_t>(kSyncHigh + kSyncLow + byte);
                    }
                }
            } else if (rx_count < static_cast<size_t>(rx_buffer[2] - 1)) {
                rx_buffer[rx_count] = byte;
                ++rx_count;
                rx_checksum = static_cast<uint8_t>(rx_checksum + byte);
            } else {
                rx_count = 0;
                if (byte == rx_checksum) {
                    handleFrame(rx_buffer);
                } else {
                    countRxError("checksum mismatch");
                }
            }
        }
    }

    void readerLoop() {
        int watched_fd = -1;
        uint64_t watched_ticket = reopen_ticket.load();
        while (running) {
            const uint64_t ticket = reopen_ticket.load();
            const int fd = port_fd.load();
            if (fd != watched_fd || ticket != watched_ticket) {
                // Abandon the previous descriptor: close first so a racing
                // open() can never hand out the same number while port_fd
                // still names it, then publish the vacancy.
                if (watched_fd >= 0) {
                    const int closing = watched_fd;
                    watched_fd = -1;
                    ::close(closing);
                    int expected = closing;
                    (void)port_fd.compare_exchange_strong(expected, -1);
                }
                watched_fd = fd;
                watched_ticket = ticket;
                rx_count = 0;  // fresh link: restart the receive parser
                continue;
            }
            if (fd < 0) {
                std::unique_lock<std::mutex> lock(wait_mutex);
                wake.wait_for(lock, Ms(20), [this] { return !running.load(); });
                continue;
            }
            struct pollfd descriptor;
            descriptor.fd = fd;
            descriptor.events = POLLIN;
            descriptor.revents = 0;
            const int ready = ::poll(&descriptor, 1, 100);
            if (!running.load()) break;
            if (ready < 0) {
                if (errno == EINTR) continue;
                linkDown(std::string("R750 serial poll failed: ") + std::strerror(errno));
                continue;
            }
            if (ready == 0) continue;
            uint8_t buffer[256];
            ssize_t got;
            do {
                got = ::read(fd, buffer, sizeof buffer);
            } while (got < 0 && errno == EINTR);
            if (!running.load()) break;
            if (got < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                linkDown(std::string("R750 serial read failed: ") + std::strerror(errno));
                continue;
            }
            if (got == 0) {  // EOF: unlike the driver, treat this as link-down
                linkDown("R750 serial link closed by the device");
                continue;
            }
            feed(buffer, static_cast<size_t>(got));
        }
    }

    void controlLoop() {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / options.cmd_vel_rate_hz));
        auto next = Clock::now();
        Clock::time_point previous{};
        while (running) {
            // Schedule from this attempt: no backlog and no extra period after
            // a slow write.
            next = Clock::now() + period;
            {
                std::lock_guard<std::mutex> io_lock(io);
                double linear = 0.0, angular = 0.0;
                bool active = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    active = running.load() && options.enable_motion_output && cached.connected &&
                             cached.control_owned && !local_estop;
                    // Stale input zeroes the output instead of replaying it.
                    if (active && command_time != Clock::time_point{} &&
                        Clock::now() - command_time <= Ms(options.cmd_vel_timeout_ms)) {
                        linear = vx;
                        angular = wz;
                    }
                }
                if (active) {
                    const auto started = Clock::now();
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        move_starts.push_back(started);
                        while (!move_starts.empty() &&
                               started - move_starts.front() > std::chrono::seconds(5)) {
                            move_starts.pop_front();
                        }
                    }
                    const auto frame = encodeVelocityFrame(linear, angular);
                    const int fd = port_fd.load();
                    const BackendResult write = fd >= 0
                        ? writeAll(fd, frame.data(), frame.size())
                        : BackendResult{false, "R750 serial port closed"};
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        if (previous != Clock::time_point{}) {
                            last_move_interval_ms =
                                std::chrono::duration<double, std::milli>(started - previous).count();
                        }
                        previous = started;
                        if (write.success) {
                            ++tx_frames;
                            // The wire has no acknowledgement; the written
                            // command is the best known device input.
                            cached.vx = linear;
                            cached.vy = 0.0;
                            cached.vyaw = angular;
                            cached.motion_state = linear == 0.0 && angular == 0.0
                                                      ? BackendMotionState::STANDING
                                                      : BackendMotionState::MOVING;
                            last_move_error.clear();
                        } else {
                            ++move_failures;
                            last_move_error = write.message;
                            clearCommandLocked();  // never replay a failed command
                        }
                    }
                    if (!write.success) linkDown(write.message);
                }
            }
            std::unique_lock<std::mutex> lock(wait_mutex);
            wake.wait_until(lock, next, [this] { return !running.load(); });
        }
    }

    // Watches telemetry freshness and reopens the port after link loss. Unlike
    // the tarkbot driver, a broken link is always retried; unlike a reconnect,
    // it never re-arms motion output.
    void monitorLoop() {
        auto next_reconnect = Clock::now() + Ms(options.reconnect_interval_ms);
        bool awaiting_close = false;
        bool port_was_open = false;
        while (running) {
            const auto now = Clock::now();
            const int fd = port_fd.load();
            if (fd >= 0) {
                port_was_open = true;
                if (!awaiting_close) {
                    Clock::time_point reference{};
                    bool connected = false;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        connected = cached.connected;
                        reference = telemetry_time != Clock::time_point{} ? telemetry_time
                                                                          : port_open_time;
                    }
                    if (now - reference > Ms(options.telemetry_timeout_ms)) {
                        linkDown(connected ? "R750 telemetry timed out; link considered lost"
                                           : "R750 received no telemetry after opening the port");
                        // The reopen wait counts from the moment the link is
                        // judged down, not from the last successful open.
                        next_reconnect = now + Ms(options.reconnect_interval_ms);
                        awaiting_close = true;
                    }
                }
            } else {
                if (port_was_open) {
                    // Same reopen wait for link failures the reader detected
                    // itself (read error/EOF/write failure) and closed for:
                    // the delay anchors to the observed link-down, so the
                    // disconnected state stays observable before reopening.
                    port_was_open = false;
                    next_reconnect = now + Ms(options.reconnect_interval_ms);
                }
                awaiting_close = false;
                if (now >= next_reconnect) {
                    std::lock_guard<std::mutex> io_lock(io);
                    if (running.load()) (void)openPort();
                    next_reconnect = Clock::now() + Ms(options.reconnect_interval_ms);
                }
            }
            std::unique_lock<std::mutex> lock(wait_mutex);
            wake.wait_for(lock, Ms(20), [this] { return !running.load(); });
        }
    }

    BackendResult ready() {
        if (!options.enable_motion_output) return {false, "R750 diagnostic mode: motion output disabled"};
        std::lock_guard<std::mutex> io_lock(io);
        bool connected = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!running.load()) return {false, "R750 backend stopped"};
            local_estop = false;
            want_control = true;
            clearCommandLocked();
            connected = cached.connected;
            cached.control_owned = connected;
            cached.motion_state = connected ? BackendMotionState::STANDING : BackendMotionState::UNKNOWN;
        }
        notifyState();
        if (!connected) return {true, "R750 armed; motion output starts once telemetry connects"};
        const BackendResult result = sendZeroOnce();
        if (!result.success) {
            linkDown(result.message);
            return result;
        }
        return {true, "R750 ready: zero velocity sent; the protocol provides no device acknowledgement"};
    }

    BackendResult stop(bool emergency) {
        if (!options.enable_motion_output) return {false, "R750 diagnostic mode: control requests disabled"};
        bool connected = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            want_control = false;
            local_estop = local_estop || emergency;
            cached.control_owned = false;
            cached.motion_state = local_estop ? BackendMotionState::ESTOP : BackendMotionState::LOCKED;
            connected = cached.connected;
            clearCommandLocked();
        }
        notifyState();
        std::lock_guard<std::mutex> io_lock(io);
        if (!running.load()) return {false, "R750 backend stopped"};
        if (!connected) {
            return {true, emergency ? "R750 software estop latched; serial link is down, nothing to send"
                                    : "R750 motion inhibited; serial link is down, nothing to send"};
        }
        const BackendResult result = sendZeroOnce();
        if (!result.success) {
            linkDown(result.message);
            return result;
        }
        return {true, emergency ? "R750 software estop latched; zero velocity sent; only ready/stand clears it"
                                : "R750 zero velocity sent; motion inhibited until ready/stand"};
    }
};

R750Backend::R750Backend(R750Options options) : impl_(std::make_unique<Impl>(std::move(options))) {}
R750Backend::~R750Backend() { (void)disconnect(); }

BackendResult R750Backend::connect() {
    auto &p = *impl_;
    if (p.running.exchange(true)) return {true, "R750 workers already running"};
    BackendResult result;
    {
        std::lock_guard<std::mutex> io_lock(p.io);
        result = p.openPort();
    }
    p.reader_thread = std::thread([&p] { p.readerLoop(); });
    p.control_thread = std::thread([&p] { p.controlLoop(); });
    p.monitor_thread = std::thread([&p] { p.monitorLoop(); });
    if (result.success) {
        // Give callers a meaningful initial status; the monitor keeps retrying
        // in the background regardless of the outcome.
        std::unique_lock<std::mutex> lock(p.wait_mutex);
        p.wake.wait_for(lock, Ms(p.options.connect_timeout_ms), [&p] {
            std::lock_guard<std::mutex> state(p.mutex);
            return p.cached.connected;
        });
        std::lock_guard<std::mutex> state(p.mutex);
        if (p.cached.connected) {
            result = {true, "R750 connected: telemetry streaming from " + p.options.serial_port};
        } else {
            result = {false, "R750 received no telemetry within connect_timeout_ms; retrying in background"};
        }
    }
    return result;
}

BackendResult R750Backend::disconnect() {
    auto &p = *impl_;
    if (!p.running.exchange(false)) return {true, "R750 already stopped"};
    p.wake.notify_all();
    if (p.control_thread.joinable()) p.control_thread.join();
    if (p.monitor_thread.joinable()) p.monitor_thread.join();
    if (p.reader_thread.joinable()) p.reader_thread.join();
    // Workers are gone: the reader leaves the port open on exit so teardown can
    // still deliver the tarkbot-driver style zero-velocity stop frame.
    BackendResult result{true, "R750 query-only backend stopped without control requests"};
    {
        std::lock_guard<std::mutex> io_lock(p.io);
        const int fd = p.port_fd.exchange(-1);
        if (fd >= 0) {
            if (p.options.enable_motion_output) {
                const auto frame = encodeVelocityFrame(0.0, 0.0);
                const BackendResult write = p.writeAll(fd, frame.data(), frame.size());
                if (write.success) {
                    std::lock_guard<std::mutex> lock(p.mutex);
                    ++p.tx_frames;
                    p.cached.vx = p.cached.vy = p.cached.vyaw = 0.0;
                    result = {true, "R750 stopped; zero velocity sent"};
                } else {
                    result = write;
                }
            }
            ::close(fd);
        } else if (p.options.enable_motion_output) {
            result = {true, "R750 stopped; serial port was already closed"};
        }
    }
    {
        std::lock_guard<std::mutex> lock(p.mutex);
        p.cached.connected = false;
        p.cached.control_owned = false;
        p.cached.battery_percent = -1.0;
        p.cached.motion_state = p.local_estop ? BackendMotionState::ESTOP : BackendMotionState::UNKNOWN;
    }
    p.notifyState();
    return result;
}

BackendResult R750Backend::takeControl() { return impl_->ready(); }
BackendResult R750Backend::stand() { return impl_->ready(); }
BackendResult R750Backend::lie() { return impl_->stop(false); }
BackendResult R750Backend::releaseControl() { return impl_->stop(false); }
BackendResult R750Backend::softEstop(bool enabled) { return enabled ? impl_->stop(true) : impl_->ready(); }
BackendResult R750Backend::setMode(int) {
    return {false, "R750 gait/mode switching is unsupported; differential chassis"};
}
BackendResult R750Backend::setSpeed(int) {
    return {false, "R750 has no speed gears; configure max_linear_speed/max_angular_speed"};
}

BackendResult R750Backend::move(double velocity_x, double velocity_y, double velocity_yaw) {
    auto &p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    if (!std::isfinite(velocity_x) || !std::isfinite(velocity_y) || !std::isfinite(velocity_yaw)) {
        p.clearCommandLocked();
        return {false, "R750 rejected non-finite velocity; previous command cleared"};
    }
    if (!p.running.load() || !p.cached.connected || !p.cached.control_owned || p.local_estop) {
        return {false, "R750 motion inhibited; call ready/stand before sending fresh cmd_vel"};
    }
    p.vx = std::clamp(velocity_x, -p.options.max_linear_speed, p.options.max_linear_speed);
    p.wz = std::clamp(velocity_yaw, -p.options.max_angular_speed, p.options.max_angular_speed);
    p.command_time = Clock::now();
    return {true, "R750 latest velocity cached (lateral velocity ignored)"};
}

BackendState R750Backend::state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->cached;
}

std::string R750Backend::diagnostics() const {
    const auto &p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    const auto now = Clock::now();
    const bool fresh = p.telemetry_time != Clock::time_point{} &&
                       now - p.telemetry_time <= Ms(p.options.telemetry_timeout_ms);
    size_t rate_samples = 0;
    Clock::time_point first{}, last{};
    for (const auto &start : p.move_starts) {
        if (now - start > std::chrono::seconds(5)) continue;
        if (rate_samples++ == 0) first = start;
        last = start;
    }
    const double actual_rate = rate_samples > 1
        ? static_cast<double>(rate_samples - 1) /
              std::chrono::duration<double>(last - first).count()
        : 0.0;
    const auto age = [&now](Clock::time_point stamp) {
        return stamp == Clock::time_point{}
            ? -1.0
            : std::chrono::duration<double, std::milli>(now - stamp).count();
    };
    const std::vector<std::string> fields = {
        "\"robot_type\": \"r750\"",
        "\"serial_port\": " + escapeJson(p.options.serial_port),
        "\"serial_baud\": " + std::to_string(p.options.serial_baud),
        "\"connected\": " + std::string(p.cached.connected ? "true" : "false"),
        "\"control_owned\": " + std::string(p.cached.control_owned ? "true" : "false"),
        // Compatibility field above is local permission, never device arbitration.
        "\"bridge_output_enabled\": " + std::string(p.cached.control_owned ? "true" : "false"),
        "\"control_owner_known\": false",
        "\"control_owner\": \"unknown\"",
        "\"control_owner_source\": \"not_reported_by_device\"",
        "\"robot_controlled_by_bridge\": null",
        "\"device_state\": \"serial_telemetry\"",
        "\"enable_motion_output\": " + std::string(p.options.enable_motion_output ? "true" : "false"),
        "\"local_estop\": " + std::string(p.local_estop ? "true" : "false"),
        "\"cmd_vel_rate_hz\": " + jsonNumber(p.options.cmd_vel_rate_hz),
        "\"last_move_interval_ms\": " + jsonNumber(p.last_move_interval_ms),
        "\"actual_move_rate_hz\": " + jsonNumber(actual_rate),
        "\"move_rate_samples\": " + std::to_string(rate_samples),
        "\"move_failures\": " + std::to_string(p.move_failures),
        "\"tx_frames\": " + std::to_string(p.tx_frames),
        "\"rx_frames\": " + std::to_string(p.rx_frames),
        "\"rx_errors\": " + std::to_string(p.rx_errors),
        "\"last_move_error\": " + escapeJson(p.last_move_error),
        "\"battery_voltage\": " + jsonNumber(p.battery_voltage),
        "\"battery_percent\": " + jsonNumber(p.cached.battery_percent),
        "\"state_age_ms\": " + jsonNumber(age(p.telemetry_time)),
        "\"cmd_vel_age_ms\": " + jsonNumber(age(p.command_time)),
        "\"cmd_vel_fresh\": " + std::string(p.command_time != Clock::time_point{} &&
                                                   now - p.command_time <= Ms(p.options.cmd_vel_timeout_ms)
                                               ? "true"
                                               : "false"),
        "\"raw_vel_valid\": " + std::string(p.cached.connected && fresh ? "true" : "false"),
        "\"raw_vel_age_ms\": " + jsonNumber(age(p.telemetry_time)),
        "\"last_raw_vel_error\": \"\"",
        "\"last_error\": " + escapeJson(p.last_error),
        "\"velocity_source\": \"last_sent_command\"",
    };
    std::string report = "{";
    for (size_t i = 0; i < fields.size(); ++i) {
        if (i != 0) report += ", ";
        report += fields[i];
    }
    return report + "}";
}

void R750Backend::setStateCallback(StateCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->state_callback = std::move(callback);
}
void R750Backend::setFaultCallback(FaultCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->fault_callback = std::move(callback);
}
void R750Backend::setRawVelocityCallback(RawVelocityCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->raw_velocity_callback = std::move(callback);
}
// R750 publishes measured velocity only; the IMU block is parsed past and no
// odometry is fabricated.
void R750Backend::setImuCallback(ImuCallback) {}
void R750Backend::setOdometryCallback(OdometryCallback) {}
void R750Backend::setJointCallback(JointCallback) {}

}  // namespace nav_bridge
