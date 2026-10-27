#include "nav_bridge/tank/tank_backend.hpp"

#include <curl/curl.h>
#include <json/json.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace nav_bridge {
namespace {
using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

struct CurlRuntime {
    CurlRuntime() {
        if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
            throw std::runtime_error("Unable to initialize libcurl");
        }
    }
    ~CurlRuntime() { curl_global_cleanup(); }
};

struct HttpSession {
    CURL *handle{curl_easy_init()};
    bool busy{false};
    ~HttpSession() { if (handle) curl_easy_cleanup(handle); }
};

size_t receiveBody(char *data, size_t size, size_t count, void *target) {
    auto &body = *static_cast<std::string *>(target);
    const size_t length = size * count;
    if (length > 65536 || body.size() > 65536 - length) return 0;
    body.append(data, length);
    return length;
}

std::string encode(const Json::Value &value) {
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    return Json::writeString(writer, value);
}

void validate(const TankOptions &o) {
    if (!std::isfinite(o.raw_vel_poll_rate_hz) || o.raw_vel_poll_rate_hz <= 0.0 ||
        o.raw_vel_poll_rate_hz > 100.0 || o.raw_vel_timeout_ms <= 0) {
        throw std::invalid_argument("TANK raw_vel_poll_rate_hz must be in (0,100], raw_vel_timeout_ms > 0");
    }
    if (o.host_ip.empty() || o.host_ip.find_first_of("/\\?#@ \t\r\n") != std::string::npos ||
        o.host_port < 1 || o.host_port > 65535) {
        throw std::invalid_argument("TANK requires a host name/IP and port in 1..65535");
    }
    if (!std::isfinite(o.cmd_vel_rate_hz) || o.cmd_vel_rate_hz < 3.0 ||
        o.cmd_vel_rate_hz > 100.0) {
        throw std::invalid_argument("TANK cmd_vel_rate_hz must be in 3..100 (protocol minimum 3 Hz)");
    }
    if (!std::isfinite(o.max_linear_speed) || o.max_linear_speed <= 0.0 ||
        o.max_linear_speed > 1.2 || !std::isfinite(o.max_angular_speed) ||
        o.max_angular_speed <= 0.0) {
        throw std::invalid_argument("TANK speed limits: linear in (0, 1.2] m/s, angular > 0 rad/s");
    }
    if (o.connect_timeout_ms <= 0 || o.http_timeout_ms <= 0 || o.move_timeout_ms <= 0 ||
        o.move_failure_limit <= 0 || o.state_failure_limit <= 0 || o.move_failure_timeout_ms <= 0 ||
        o.reconnect_interval_ms <= 0 || o.cmd_vel_timeout_ms <= 0 ||
        o.state_poll_interval_ms <= 0 || o.battery_poll_interval_ms <= 0 ||
        o.telemetry_timeout_ms <= o.state_poll_interval_ms + 2 * o.http_timeout_ms) {
        throw std::invalid_argument("TANK timeouts and failure limits must be positive; telemetry_timeout_ms must exceed "
                                    "state_poll_interval_ms + 2 * http_timeout_ms");
    }
}
}  // namespace

struct TankBackend::Impl {
    explicit Impl(TankOptions value) : options(std::move(value)) {
        validate(options);
        static CurlRuntime runtime;
        (void)runtime;
        want_control = options.startup_acquire_control;
    }

    TankOptions options;
    mutable std::mutex mutex;
    // Services/reconnection serialize with each other; motion I/O serializes
    // with action I/O. Telemetry has its own HTTP handles and never holds io.
    std::mutex action_mutex, io;
    std::mutex wait_mutex;
    std::condition_variable wake;
    std::atomic<bool> running{false};
    std::atomic<uint64_t> epoch{0};
    std::thread control_thread, telemetry_thread, raw_velocity_thread;
    BackendState cached;
    bool want_control{false};
    bool local_estop{false};
    bool brake{false};
    bool brake_known{false};
    bool initialized{false};
    bool initialized_known{false};
    std::string operation, last_error;
    double vx{0.0}, wz{0.0};
    Clock::time_point command_time{}, telemetry_time{};
    uint64_t move_failures{0};
    double last_move_interval_ms{0.0}, last_request_ms{0.0};
    int consecutive_move_failures{0}, consecutive_state_failures{0};
    Clock::time_point last_move_ack_time{};
    std::deque<Clock::time_point> move_starts;
    std::string last_move_error, last_state_error, last_battery_error;
    mutable std::mutex http_mutex;
    mutable std::vector<std::unique_ptr<HttpSession>> http_sessions;
    StateCallback state_callback;
    FaultCallback fault_callback;
    RawVelocityCallback raw_velocity_callback;
    Clock::time_point raw_velocity_time{};
    bool raw_velocity_valid{false};
    std::string last_raw_velocity_error;

    BackendResult request(const std::string &path, const Json::Value *payload,
                          int timeout_ms, Json::Value &response, bool *transport_failure = nullptr) const {
        if (transport_failure) *transport_failure = false;
        if (!options.enable_motion_output && path != "/robot/cmd/get_robot_state" &&
            path != "/robot/cmd/get_move_brake" && path != "/robot/sensor_data/battery" &&
            path != "/robot/sensor_data/raw_velocity") {
            return {false, "TANK diagnostic mode: control requests disabled"};
        }
        // Lease a handle: concurrent telemetry never shares a CURL easy handle
        // with motion/actions. reset retains its connection cache for reuse.
        HttpSession *session = nullptr;
        {
            std::lock_guard<std::mutex> lock(http_mutex);
            for (auto &candidate : http_sessions) {
                if (!candidate->busy) { session = candidate.get(); break; }
            }
            if (!session) {
                http_sessions.push_back(std::make_unique<HttpSession>());
                session = http_sessions.back().get();
            }
            session->busy = true;
        }
        struct Lease {
            const Impl &owner;
            HttpSession *session;
            ~Lease() {
                if (session->handle) curl_easy_reset(session->handle);
                std::lock_guard<std::mutex> lock(owner.http_mutex);
                session->busy = false;
            }
        } lease{*this, session};
        CURL *handle = session->handle;
        if (!handle) return {false, "Unable to create HTTP client"};
        const std::string url = "http://" + options.host_ip + ":" +
                                std::to_string(options.host_port) + path;
        const std::string body = payload ? encode(*payload) : std::string();
        std::string received;
        char error[CURL_ERROR_SIZE] = {};
        curl_slist *headers = curl_slist_append(nullptr, "Content-Type: application/json");
        curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
        curl_easy_setopt(handle, CURLOPT_PROXY, "");
        curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT_MS,
                         static_cast<long>(std::min(options.connect_timeout_ms, timeout_ms)));
        curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_ms));
        curl_easy_setopt(handle, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(handle, CURLOPT_POST, 1L);
        curl_easy_setopt(handle, CURLOPT_POSTFIELDS, body.c_str());
        curl_easy_setopt(handle, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
        curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, receiveBody);
        curl_easy_setopt(handle, CURLOPT_WRITEDATA, &received);
        curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error);
        const auto code = curl_easy_perform(handle);
        long status = 0;
        curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &status);
        curl_slist_free_all(headers);
        if (code != CURLE_OK) {
            if (transport_failure) {
                *transport_failure = code == CURLE_OPERATION_TIMEDOUT || code == CURLE_COULDNT_CONNECT ||
                    code == CURLE_COULDNT_RESOLVE_HOST || code == CURLE_RECV_ERROR ||
                    code == CURLE_SEND_ERROR || code == CURLE_GOT_NOTHING || code == CURLE_PARTIAL_FILE;
            }
            return {false, path + ": " + (error[0] ? error : curl_easy_strerror(code))};
        }
        if (status < 200 || status >= 300) {
            // Keep bounded device errors and escape multiline server tracebacks.
            const std::string detail = received.empty() ? "" :
                ", response=" + encode(Json::Value(received.substr(0, 1024))) +
                (received.size() > 1024 ? " (truncated)" : "");
            return {false, path + ": HTTP " + std::to_string(status) + detail};
        }
        Json::CharReaderBuilder builder;
        builder["failIfExtra"] = true;
        std::string errors;
        std::istringstream stream(received);
        if (!Json::parseFromStream(builder, stream, &response, &errors) || !response.isObject()) {
            return {false, path + ": invalid JSON response"};
        }
        if (!response["successed"].isBool() || !response["successed"].asBool()) {
            return {false, path + ": rejected: errorCode=" + encode(response["errorCode"]) +
                           ", msg=" + encode(response["msg"])};
        }
        return {true, path + ": acknowledged"};
    }

    BackendResult sendMove(double linear, double angular, bool *transport_failure = nullptr) const {
        Json::Value body, response;
        body["speed"]["linearSpeed"] = linear;
        // This TANK's observed steering follows ROS yaw; do not invert it.
        // The supplied API document's right-positive convention differs.
        body["speed"]["angularSpeed"] = angular;
        return request("/robot/cmd/move", &body, options.move_timeout_ms, response, transport_failure);
    }

    BackendResult queryState(std::string &state, bool *transport_failure = nullptr) {
        Json::Value response;
        auto result = request("/robot/cmd/get_robot_state", nullptr, options.http_timeout_ms, response, transport_failure);
        if (!result.success) return result;
        const auto &data = response["data"];
        if (!data.isObject() || !data["state_operation"].isString() ||
            data["state_operation"].asString().empty()) {
            return {false, "Robot state_operation missing or invalid"};
        }
        // The real device reports Idle + successed=true with initialized=false.
        // This flag is diagnostic metadata, not proof that motion is unavailable.
        {
            std::lock_guard<std::mutex> lock(mutex);
            initialized_known = data["state_initialized"].isBool();
            initialized = initialized_known && data["state_initialized"].asBool();
        }
        state = data["state_operation"].asString();
        return {true, "Robot state confirmed"};
    }

    BackendResult queryBrake(bool &value, bool *transport_failure = nullptr) const {
        Json::Value response;
        auto result = request("/robot/cmd/get_move_brake", nullptr, options.http_timeout_ms, response, transport_failure);
        if (!result.success) return result;
        if (!response["data"].isObject() || !response["data"]["brake_state"].isBool()) {
            return {false, "Robot brake_state missing or invalid"};
        }
        value = response["data"]["brake_state"].asBool();
        return {true, "Brake state confirmed"};
    }

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
        vx = wz = cached.vx = cached.vy = cached.vyaw = 0.0;
        command_time = Clock::time_point{};
    }

    void linkFailed(const std::string &message, uint64_t ticket) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (ticket != epoch.load()) return;
            cached.connected = cached.control_owned = false;
            cached.battery_percent = -1.0;
            cached.motion_state = local_estop ? BackendMotionState::ESTOP : BackendMotionState::UNKNOWN;
            brake_known = false;
            clearCommandLocked();
            ++epoch;
        }
        reportError(message);
        notifyState();
        wake.notify_all();
    }

    // Called with action_mutex + io held. An explicit ready may release the
    // brake; automatic reconnection only observes it, never clears it.
    BackendResult prepare(bool explicit_ready, uint64_t ticket) {
        Json::Value response;
        BackendResult result{true, "Query-only connection"};
        if (options.enable_motion_output) {
            result = request("/robot/cmd/change_robot_state?state=Idle", nullptr,
                             options.http_timeout_ms, response);
        }
        std::string device_state;
        bool device_brake = true;
        if (result.success) result = queryState(device_state);
        if (result.success && options.enable_motion_output && device_state != "Idle") {
            result = {false, "Robot did not confirm Idle"};
        }
        if (result.success) result = queryBrake(device_brake);
        // Ready is idempotent: an already released brake needs no write.
        // A real release still requires readback confirmation.
        if (result.success && explicit_ready && device_brake && ticket == epoch.load()) {
            Json::Value body;
            body["brake"] = false;
            result = request("/robot/cmd/move_brake", &body, options.http_timeout_ms, response);
            if (result.success) result = queryBrake(device_brake);
        }
        if (result.success && explicit_ready && device_brake) {
            result = {false, "Robot did not confirm software brake release"};
        }
        if (result.success && options.enable_motion_output) result = sendMove(0.0, 0.0);
        if (!result.success) {
            linkFailed(result.message, ticket);
            return result;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!running || ticket != epoch.load()) return {false, "Preparation superseded by stop/shutdown"};
            operation = device_state;
            brake = device_brake;
            brake_known = true;
            if (explicit_ready) {
                local_estop = false;
                want_control = true;
            }
            cached.connected = true;
            cached.control_owned = options.enable_motion_output && want_control && !brake && !local_estop;
            cached.motion_state = (brake || local_estop) ? BackendMotionState::ESTOP :
                (cached.control_owned ? BackendMotionState::STANDING : BackendMotionState::LOCKED);
            telemetry_time = Clock::now();
            last_error.clear();
            consecutive_move_failures = consecutive_state_failures = 0;
            last_move_error.clear();
            last_state_error.clear();
            last_move_ack_time = Clock::now();
            clearCommandLocked();
        }
        notifyState();
        if (!options.enable_motion_output) return {true, "TANK query-only connection confirmed; motion output disabled"};
        return {true, explicit_ready ? "TANK ready: Idle and brake release confirmed; waiting for fresh cmd_vel" :
                                     "TANK connected: Idle and brake state confirmed"};
    }

    BackendResult ready() {
        if (!options.enable_motion_output) return {false, "TANK diagnostic mode: motion output disabled"};
        const auto ticket = inhibit(false);
        std::lock_guard<std::mutex> action_lock(action_mutex);
        std::lock_guard<std::mutex> io_lock(io);
        if (!running || ticket != epoch.load()) return {false, "Ready superseded or backend stopped"};
        return prepare(true, ticket);
    }

    uint64_t inhibit(bool emergency) {
        std::lock_guard<std::mutex> lock(mutex);
        const auto ticket = ++epoch;
        want_control = false;
        cached.control_owned = false;
        local_estop = local_estop || emergency;
        cached.motion_state = local_estop ? BackendMotionState::ESTOP : BackendMotionState::LOCKED;
        clearCommandLocked();
        return ticket;
    }

    BackendResult stop(bool emergency) {
        if (!options.enable_motion_output) return {false, "TANK diagnostic mode: control requests disabled"};
        const auto ticket = inhibit(emergency);
        notifyState();
        std::lock_guard<std::mutex> action_lock(action_mutex);
        std::lock_guard<std::mutex> io_lock(io);
        if (!running || ticket != epoch.load()) return {false, "Stop superseded or backend stopped"};
        BackendResult result;
        if (emergency) {
            Json::Value body, response;
            body["brake"] = true;
            result = request("/robot/cmd/move_brake", &body, options.http_timeout_ms, response);
            bool confirmed = false;
            if (result.success) result = queryBrake(confirmed);
            if (result.success && !confirmed) result = {false, "Robot did not confirm software brake activation"};
            if (result.success) {
                std::lock_guard<std::mutex> lock(mutex);
                brake = brake_known = true;
            }
        } else {
            result = sendMove(0.0, 0.0);
        }
        if (!result.success) {
            linkFailed(result.message, ticket);
            return result;
        }
        notifyState();
        return {true, emergency ? "TANK software brake confirmed; motion inhibited" :
                                 "TANK zero velocity acknowledged; motion inhibited until ready/stand"};
    }

    void controlLoop() {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / options.cmd_vel_rate_hz));
        auto next = Clock::now();
        Clock::time_point previous{};
        while (running) {
            // Schedule from this attempt, without adding a full period after
            // a slow response or replaying missed deadlines.
            next = Clock::now() + period;
            {
                std::lock_guard<std::mutex> io_lock(io);
                double linear = 0.0, angular = 0.0;
                bool active, stale, ack_expired;
                uint64_t ticket;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    ticket = epoch.load();
                    active = running && options.enable_motion_output && cached.connected &&
                             cached.control_owned && !local_estop && !brake;
                    stale = active && Clock::now() - telemetry_time > Ms(options.telemetry_timeout_ms);
                    ack_expired = active && consecutive_move_failures > 0 &&
                        Clock::now() - last_move_ack_time >= Ms(options.move_failure_timeout_ms);
                    if (command_time != Clock::time_point{} &&
                        Clock::now() - command_time <= Ms(options.cmd_vel_timeout_ms)) {
                        linear = vx;
                        angular = wz;
                    }
                }
                if (stale || ack_expired) {
                    (void)sendMove(0.0, 0.0);
                    linkFailed(stale ? "Robot state telemetry expired" : "Motion acknowledgement expired", ticket);
                } else if (active) {
                    const auto started = Clock::now();
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        move_starts.push_back(started);
                        while (!move_starts.empty() && started - move_starts.front() > std::chrono::seconds(5)) {
                            move_starts.pop_front();
                        }
                    }
                    bool transport_failure = false, disconnect = false;
                    auto result = sendMove(linear, angular, &transport_failure);
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        last_request_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
                        if (previous != Clock::time_point{}) {
                            last_move_interval_ms = std::chrono::duration<double, std::milli>(started - previous).count();
                        }
                        previous = started;
                        if (!result.success) ++move_failures;
                        if (ticket == epoch.load() && cached.control_owned) {
                            if (result.success) {
                                consecutive_move_failures = 0;
                                last_move_error.clear();
                                last_move_ack_time = Clock::now();
                            } else {
                                ++consecutive_move_failures;
                                last_move_error = result.message;
                                // Never retry the failed cached velocity. Only
                                // fresh ROS input can produce another nonzero.
                                clearCommandLocked();
                                cached.motion_state = BackendMotionState::UNKNOWN;
                                disconnect = !transport_failure ||
                                    consecutive_move_failures >= options.move_failure_limit ||
                                    Clock::now() - last_move_ack_time >= Ms(options.move_failure_timeout_ms);
                            }
                        }
                        if (result.success && ticket == epoch.load() && cached.control_owned) {
                            cached.vx = linear;
                            cached.vyaw = angular;
                            cached.motion_state = (linear == 0.0 && angular == 0.0) ?
                                BackendMotionState::STANDING : BackendMotionState::MOVING;
                        }
                    }
                    if (!result.success) {
                        if (disconnect) linkFailed(result.message, ticket);
                        else reportError(result.message);
                    }
                }
            }
            std::unique_lock<std::mutex> lock(wait_mutex);
            wake.wait_until(lock, next, [this] { return !running.load(); });
        }
    }

    void rawVelocityLoop() {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / options.raw_vel_poll_rate_hz));
        // Observations remain useful during local inhibition or remote control.
        while (running) {
            const auto next = Clock::now() + period;
            Json::Value response;
            auto result = request("/robot/sensor_data/raw_velocity", nullptr,
                                  options.raw_vel_timeout_ms, response);
            const auto data = response.isObject() && response["data"].isObject() ?
                response["data"]["device_data"] : Json::Value();
            double linear = 0.0, lateral = 0.0, angular = 0.0;
            if (result.success) {
                if (!data.isObject() || !data["linear_vel"].isNumeric() ||
                    !data["linear_y"].isNumeric() || !data["angular_rotate"].isNumeric()) {
                    result = {false, "Raw velocity response missing numeric linear_vel/linear_y/angular_rotate"};
                } else {
                    linear = data["linear_vel"].asDouble();
                    lateral = data["linear_y"].asDouble();
                    angular = data["angular_rotate"].asDouble();
                    if (!std::isfinite(linear) || !std::isfinite(lateral) || !std::isfinite(angular)) {
                        result = {false, "Raw velocity response contains non-finite values"};
                    }
                }
            }
            RawVelocityCallback callback;
            {
                std::lock_guard<std::mutex> lock(mutex);
                raw_velocity_valid = result.success;
                last_raw_velocity_error = result.success ? "" : result.message;
                if (result.success) {
                    raw_velocity_time = Clock::now();
                    callback = raw_velocity_callback;
                }
            }
            if (running) {
                if (callback) callback(linear, lateral, angular);
                if (!result.success) reportError(result.message);
            }
            std::unique_lock<std::mutex> lock(wait_mutex);
            wake.wait_until(lock, next, [this] { return !running.load(); });
        }
    }

    void telemetryLoop() {
        auto next_state = Clock::now();
        auto next_battery = Clock::now();
        auto next_connect = Clock::now() + Ms(options.reconnect_interval_ms);
        while (running) {
            bool connected;
            {
                std::lock_guard<std::mutex> lock(mutex);
                connected = cached.connected;
            }
            if (!connected && Clock::now() >= next_connect) {
                std::unique_lock<std::mutex> action_lock(action_mutex, std::try_to_lock);
                if (action_lock.owns_lock()) {
                    const auto ticket = epoch.load();
                    std::lock_guard<std::mutex> io_lock(io);
                    if (running && ticket == epoch.load()) (void)prepare(false, ticket);
                }
                next_connect = Clock::now() + Ms(options.reconnect_interval_ms);
            } else if (connected && Clock::now() >= next_state) {
                const auto ticket = epoch.load();
                std::string device_state;
                bool device_brake = true;
                bool transport_failure = false;
                auto result = queryState(device_state, &transport_failure);
                if (result.success) result = queryBrake(device_brake, &transport_failure);
                // Discard responses from before a ready/stop transition.
                if (ticket == epoch.load()) {
                    if (!result.success) {
                        bool disconnect;
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            ++consecutive_state_failures;
                            disconnect = !transport_failure ||
                                consecutive_state_failures >= options.state_failure_limit ||
                                Clock::now() - telemetry_time >= Ms(options.telemetry_timeout_ms);
                            last_state_error = result.message;
                        }
                        if (disconnect) linkFailed(result.message, ticket);
                        else reportError(result.message);
                    } else {
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            if (ticket == epoch.load()) {
                                operation = device_state;
                                brake = device_brake;
                                brake_known = true;
                                telemetry_time = Clock::now();
                                consecutive_state_failures = 0;
                                last_state_error.clear();
                                if (brake || device_state != "Idle") {
                                    cached.control_owned = want_control = false;
                                    cached.motion_state = brake ? BackendMotionState::ESTOP : BackendMotionState::LOCKED;
                                    clearCommandLocked();
                                }
                            }
                        }
                        notifyState();
                    }
                }
                next_state = Clock::now() + Ms(options.state_poll_interval_ms);
            }
            if (connected && Clock::now() >= next_battery) {
                Json::Value response;
                const auto ticket = epoch.load();
                const auto result = request("/robot/sensor_data/battery", nullptr, options.http_timeout_ms, response);
                const auto value = result.success && response["data"].isObject() ?
                    response["data"]["battery_capacity_percentage"] : Json::Value();
                double percentage = -1.0;
                if (result.success && value.isNumeric() && std::isfinite(value.asDouble()) &&
                    value.asDouble() >= 0.0 && value.asDouble() <= 100.0) percentage = value.asDouble();
                const std::string battery_error = percentage >= 0.0 ? "" :
                    (result.success ? "Battery percentage missing, nonnumeric, or outside 0..100: " + encode(value) : result.message);
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (ticket == epoch.load() && cached.connected) {
                        cached.battery_percent = percentage;
                        last_battery_error = battery_error;
                    }
                }
                if (percentage < 0.0) reportError(battery_error);
                notifyState();
                next_battery = Clock::now() + Ms(options.battery_poll_interval_ms);
            }
            std::unique_lock<std::mutex> lock(wait_mutex);
            wake.wait_for(lock, Ms(20), [this] { return !running.load(); });
        }
    }
};

TankBackend::TankBackend(TankOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}
TankBackend::~TankBackend() { (void)disconnect(); }

BackendResult TankBackend::connect() {
    auto &p = *impl_;
    if (p.running.exchange(true)) return {true, "TANK workers already running"};
    BackendResult result;
    {
        std::lock_guard<std::mutex> action_lock(p.action_mutex);
        std::lock_guard<std::mutex> io_lock(p.io);
        result = p.prepare(false, p.epoch.load());
    }
    p.control_thread = std::thread([&p] { p.controlLoop(); });
    p.raw_velocity_thread = std::thread([&p] { p.rawVelocityLoop(); });
    p.telemetry_thread = std::thread([&p] { p.telemetryLoop(); });
    return result;
}

BackendResult TankBackend::disconnect() {
    auto &p = *impl_;
    if (!p.running.exchange(false)) return {true, "TANK already stopped"};
    p.inhibit(false);
    p.wake.notify_all();
    if (p.control_thread.joinable()) p.control_thread.join();
    if (p.telemetry_thread.joinable()) p.telemetry_thread.join();
    if (p.raw_velocity_thread.joinable()) p.raw_velocity_thread.join();
    BackendResult result;
    {
        std::lock_guard<std::mutex> action_lock(p.action_mutex);
        std::lock_guard<std::mutex> io_lock(p.io);
        result = p.options.enable_motion_output ? p.sendMove(0.0, 0.0) :
            BackendResult{true, "TANK query-only backend stopped without control requests"};
    }
    {
        std::lock_guard<std::mutex> lock(p.mutex);
        p.cached.connected = false;
        p.cached.battery_percent = -1.0;
    }
    p.notifyState();
    return result;
}

BackendResult TankBackend::takeControl() { return impl_->ready(); }
BackendResult TankBackend::stand() { return impl_->ready(); }
BackendResult TankBackend::lie() { return impl_->stop(false); }
BackendResult TankBackend::releaseControl() { return impl_->stop(false); }
BackendResult TankBackend::softEstop(bool enabled) { return enabled ? impl_->stop(true) : impl_->ready(); }
BackendResult TankBackend::setMode(int) { return {false, "TANK gait/mode switching is unsupported; ready prepares Idle"}; }
BackendResult TankBackend::setSpeed(int) { return {false, "TANK has no speed gears; configure max_linear_speed/max_angular_speed"}; }

BackendResult TankBackend::move(double vx, double vy, double vyaw) {
    auto &p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vyaw)) {
        p.clearCommandLocked();
        return {false, "TANK rejected non-finite velocity; previous command cleared"};
    }
    if (!p.running || !p.cached.connected || !p.cached.control_owned || p.local_estop || p.brake) {
        return {false, "TANK motion inhibited; call ready/stand before sending fresh cmd_vel"};
    }
    p.vx = std::clamp(vx, -p.options.max_linear_speed, p.options.max_linear_speed);
    p.wz = std::clamp(vyaw, -p.options.max_angular_speed, p.options.max_angular_speed);
    p.command_time = Clock::now();
    return {true, "TANK latest velocity cached (lateral velocity ignored)"};
}

BackendState TankBackend::state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->cached;
}

std::string TankBackend::diagnostics() const {
    const auto &p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    Json::Value info;
    info["robot_type"] = "tank";
    info["connected"] = p.cached.connected;
    info["control_owned"] = p.cached.control_owned;
    // Compatibility field above is local permission, never device arbitration.
    info["bridge_output_enabled"] = p.cached.control_owned;
    info["control_owner_known"] = false;
    info["control_owner"] = "unknown";
    info["control_owner_source"] = "not_reported_by_device";
    info["robot_controlled_by_bridge"] = Json::Value();
    info["device_state"] = p.operation;
    info["enable_motion_output"] = p.options.enable_motion_output;
    info["state_initialized_known"] = p.initialized_known;
    info["state_initialized"] = p.initialized;
    info["brake_known"] = p.brake_known;
    info["software_brake"] = p.brake;
    info["local_estop"] = p.local_estop;
    info["cmd_vel_rate_hz"] = p.options.cmd_vel_rate_hz;
    info["last_move_interval_ms"] = p.last_move_interval_ms;
    info["last_move_request_ms"] = p.last_request_ms;
    info["move_failures"] = Json::UInt64(p.move_failures);
    info["consecutive_move_failures"] = p.consecutive_move_failures;
    info["consecutive_state_failures"] = p.consecutive_state_failures;
    info["last_move_error"] = p.last_move_error;
    info["last_state_error"] = p.last_state_error;
    info["last_battery_error"] = p.last_battery_error;
    const auto now = Clock::now();
    info["raw_vel_valid"] = p.running && p.raw_velocity_valid &&
        now - p.raw_velocity_time <= Ms(p.options.raw_vel_timeout_ms) +
            std::chrono::duration<double>(1.0 / p.options.raw_vel_poll_rate_hz);
    info["raw_vel_age_ms"] = p.raw_velocity_time == Clock::time_point{} ? -1.0 :
        std::chrono::duration<double, std::milli>(now - p.raw_velocity_time).count();
    info["raw_vel_poll_rate_hz"] = p.options.raw_vel_poll_rate_hz;
    info["last_raw_vel_error"] = p.last_raw_velocity_error;
    info["cmd_vel_age_ms"] = p.command_time == Clock::time_point{} ? -1.0 :
        std::chrono::duration<double, std::milli>(now - p.command_time).count();
    info["cmd_vel_fresh"] = p.command_time != Clock::time_point{} &&
        now - p.command_time <= Ms(p.options.cmd_vel_timeout_ms);
    size_t count = 0;
    Clock::time_point first{}, last{};
    for (const auto &start : p.move_starts) {
        if (now - start > std::chrono::seconds(5)) continue;
        if (count++ == 0) first = start;
        last = start;
    }
    info["actual_move_rate_hz"] = count > 1 ?
        (count - 1) / std::chrono::duration<double>(last - first).count() : 0.0;
    info["move_rate_samples"] = Json::UInt64(count);
    info["move_ack_age_ms"] = p.last_move_ack_time == Clock::time_point{} ? -1.0 :
        std::chrono::duration<double, std::milli>(now - p.last_move_ack_time).count();
    info["state_age_ms"] = p.telemetry_time == Clock::time_point{} ? -1.0 :
        std::chrono::duration<double, std::milli>(now - p.telemetry_time).count();
    info["last_error"] = p.last_error;
    info["velocity_source"] = "acknowledged_command";
    return encode(info);
}

void TankBackend::setStateCallback(StateCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->state_callback = std::move(callback);
}
void TankBackend::setFaultCallback(FaultCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->fault_callback = std::move(callback);
}
void TankBackend::setRawVelocityCallback(RawVelocityCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->raw_velocity_callback = std::move(callback);
}
// TANK phase one deliberately publishes no fabricated sensor observations.
void TankBackend::setImuCallback(ImuCallback) {}
void TankBackend::setOdometryCallback(OdometryCallback) {}
void TankBackend::setJointCallback(JointCallback) {}

}  // namespace nav_bridge
