#include "nav_bridge/cyvet/cyvet_backend.hpp"
#include "nav_bridge/cyvet/motion_math.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "uniubi_motion_client/motion_high_level_client.hpp"

namespace nav_bridge {
namespace {
using Clock = std::chrono::steady_clock;
using Client = uniubi_motion_client::MotionHighLevelClient;
using Rpc = uniubi_motion_client::SystemRpcCall;
using namespace std::chrono_literals;
Json::Value parse(const std::string &text) {
    Json::CharReaderBuilder builder;
    Json::Value value;
    std::string error;
    std::istringstream stream(text);
    if (!Json::parseFromStream(builder, stream, &value, &error))
        throw std::runtime_error("Invalid robot JSON: " + error);
    return value;
}
std::string json(const Json::Value &value) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, value);
}
bool finite(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); });
}
}  // namespace

struct CyvetBackend::Impl {
    explicit Impl(CyvetOptions options) : opts(std::move(options)) {
        if (opts.device_id.empty() || opts.robot_domain_id < 0 || opts.robot_domain_id > 232 ||
            opts.connect_timeout_ms <= 0 || opts.reconnect_interval_ms <= 0 ||
            opts.action_timeout_ms <= 0 || opts.rpc_timeout_ms <= 0 || opts.lease_ms < 5000 ||
            opts.cmd_vel_timeout_ms <= 0 || opts.telemetry_timeout_ms <= 0 ||
            !std::isfinite(opts.cmd_vel_rate_hz) || opts.cmd_vel_rate_hz <= 0 ||
            opts.cmd_vel_rate_hz > 200 || (opts.lateral_sign != 1 && opts.lateral_sign != -1))
            throw std::invalid_argument("Invalid Cyvet device/network/timing parameters");
        for (const auto &limits : opts.speed_limits)
            for (double limit : limits)
                if (!std::isfinite(limit) || limit < 0)
                    throw std::invalid_argument("Cyvet speed limits must be finite and nonnegative");
        context = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions init;
        init.auto_initialize_logging(false);
        init.set_domain_id(static_cast<size_t>(opts.robot_domain_id));
        context->init(0, nullptr, init);
        rclcpp::NodeOptions node_options;
        node_options.context(context).use_global_arguments(false);
        node = std::make_shared<rclcpp::Node>("cyvet_robot_client", node_options);
        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context;
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        executor->add_node(node);
        client = std::make_unique<Client>(node, *executor, opts.service_name, opts.device_id,
                                          opts.event_topic);
        client->setConnectCallback([this](Client::HighLevelState state, Client::HighLevelError error) {
            if (state != Client::kControlled) {
                inhibit();
                std::lock_guard<std::mutex> lock(mutex);
                cached.control_owned = false;
                cached.motion_state = BackendMotionState::UNKNOWN;
            }
            if (error != Client::kNone && !(releasing && error == Client::kSessionRevoked))
                fault("Control ownership changed", error);
        });
        client->setEventCallback([this](const std::string &topic, const std::string &payload) {
            if (topic == "statistics/device_status") {
                try { updateBattery(parse(payload)); } catch (const std::exception &e) { fault(e.what()); }
            }
        });
        client->setMotionObservedCallback([this](const uniubi::msg::MotionObserved &sample) {
            onMotion(sample);
        });
        client->setSensorObservedCallback([this](const uniubi::msg::SensorObserved &sample) {
            onSensor(sample);
        });
        worker = std::thread([this] { run(); });
    }
    ~Impl() {
        inhibit();
        quitting.store(true);
        cv.notify_all();
        if (worker.joinable()) worker.join();
        client.reset();
        executor->remove_node(node);
        executor.reset();
        node.reset();
        context->shutdown("Cyvet backend stopped");
    }

    struct Job {
        std::function<BackendResult(uint64_t)> action;
        std::promise<BackendResult> result;
        uint64_t generation;
        bool priority{false};
    };
    struct Velocity { double x, y, yaw; Clock::time_point at; uint64_t generation; };
    struct Pending { int64_t id; uint64_t sequence; Clock::time_point deadline; };
    CyvetOptions opts;
    std::shared_ptr<rclcpp::Context> context;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::unique_ptr<Client> client;
    std::thread worker;
    mutable std::mutex mutex;
    std::condition_variable cv;
    std::deque<std::shared_ptr<Job>> urgent, normal;
    std::optional<Velocity> velocity;
    std::atomic<bool> quitting{false}, ready{false}, started{false}, halt_pending{false};
    std::atomic<uint64_t> generation{0};
    std::atomic<unsigned> stop_requests{0};
    BackendState cached;
    StateCallback state_callback;
    ImuCallback imu_callback;
    OdometryCallback odom_callback;
    JointCallback joint_callback;
    FaultCallback fault_callback;
    std::string last_fault, current_action;
    Clock::time_point battery_at{}, motion_reply_at{}, system_reply_at{}, next_connect{},
        next_motion{}, next_system{}, next_velocity{}, next_halt{};
    std::optional<Pending> pending_motion, pending_system;
    uint64_t query_sequence{0}, motion_stamp{0}, sensor_stamp{0};
    std::array<cyvet::AxisRange, 3> ranges{};
    std::set<std::string> actions;
    std::map<std::pair<unsigned, unsigned>, std::string> layout;
    cyvet::ContinuousOdometry continuous_odom;
    int speed{1};
    bool capabilities_valid{false}, observations_enabled{false}, velocity_active{false},
         estop_latched{false}, halt_escalated{false}, releasing{false};

    void inhibit() {
        std::lock_guard<std::mutex> lock(mutex);
        ready.store(false);
        generation.fetch_add(1);
        velocity.reset();
    }
    void fault(const std::string &message, int code = -1) {
        FaultCallback callback;
        { std::lock_guard<std::mutex> lock(mutex); last_fault = message; callback = fault_callback; }
        if (callback) callback({code, 2, message});
    }
    BackendResult error(const std::string &operation) {
        const auto code = client->getLastError();
        fault(operation, code);
        return {false, operation + " (error=" + std::to_string(code) + ")"};
    }
    BackendResult submit(std::function<BackendResult(uint64_t)> action, bool priority = false, bool prepare = false) {
        auto job = std::make_shared<Job>();
        job->action = std::move(action);
        job->priority = priority;
        auto result = job->result.get_future();
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (quitting) return {false, "Cyvet backend is stopping"};
            if (prepare && (stop_requests || halt_pending)) return {false, "A stop/release request is pending"};
            if (!priority && normal.size() + urgent.size() >= 16)
                return {false, "Cyvet command queue is full"};
            // Serialize acceptance with stop requests. A later preparation must
            // never invalidate an already queued stop, even under concurrent services.
            if (priority || prepare) {
                ready.store(false);
                generation.fetch_add(1);
                velocity.reset();
            }
            if (priority) stop_requests.fetch_add(1);
            job->generation = generation.load();
            if (priority) urgent.push_back(job); else normal.push_back(job);
        }
        cv.notify_all();
        auto response = result.get();
        if (priority) stop_requests.fetch_sub(1);
        return response;
    }
    BackendResult submitStop(std::function<BackendResult(uint64_t)> action) {
        return submit(std::move(action),true);
    }
    bool interrupted(uint64_t token) const {
        return quitting || generation.load() != token;
    }
    void updateBattery(const Json::Value &root) {
        const auto &battery = root["battery"];
        if (!battery.isObject()) return;
        const auto &power = battery["power"];
        if (!power.isNumeric()) return;
        double value = power.asDouble();
        if (!std::isfinite(value) || value < 0 || value > 100 ||
            (battery.isMember("online") && !battery["online"].asBool())) return;
        std::lock_guard<std::mutex> lock(mutex);
        cached.battery_percent = value;
        battery_at = Clock::now();
    }
    void updateMotion(const Json::Value &root) {
        if (!root.isObject()) throw std::runtime_error("Motion state must be an object");
        const std::string action = root.get("action", "").asString();
        if ((action == "walking" || action == "emergencyStop") && (!root["lineVelocityX"].isNumeric() ||
            !root["lineVelocityY"].isNumeric() || !root["velocity"].isNumeric()))
            throw std::runtime_error("Walking/emergencyStop state lacks complete three-axis velocity");
        const double x = root.get("lineVelocityX", 0.0).asDouble();
        const double y = root.get("lineVelocityY", 0.0).asDouble() * opts.lateral_sign;
        const double yaw = root.get("velocity", 0.0).asDouble();
        if (!finite({x, y, yaw})) throw std::runtime_error("Non-finite motion state");
        if (ready && action != "walking") inhibit();
        std::lock_guard<std::mutex> lock(mutex);
        current_action = action;
        cached.vx = x; cached.vy = y; cached.vyaw = yaw;
        cached.connected = true;
        cached.control_owned = client->getState() == Client::kControlled;
        cached.motion_state = estop_latched ? BackendMotionState::ESTOP :
            action == "laying" ? BackendMotionState::LYING_DOWN :
            action == "standing" ? BackendMotionState::STANDING :
            action == "walking" ? (std::abs(x)+std::abs(y)+std::abs(yaw) > 1e-5 ?
                BackendMotionState::MOVING : BackendMotionState::STANDING) : BackendMotionState::UNKNOWN;
        motion_reply_at = Clock::now();
    }
    void parseCapabilities(const Json::Value &root) {
        actions.clear(); capabilities_valid = false;
        const auto &list = root.isArray() ? root : root["actions"];
        if (!list.isArray()) throw std::runtime_error("Missing action capabilities");
        std::array<bool, 3> found{};
        const std::array<std::string, 3> names{{"lineVelocityX", "lineVelocityY", "velocity"}};
        for (const auto &action : list) {
            if (!action["name"].isString()) continue;
            actions.insert(action["name"].asString());
            if (action["name"].asString() != "walking") continue;
            for (const auto &param : action["params"]) {
                for (size_t i = 0; i < names.size(); ++i) {
                    if (param.get("name", "").asString() != names[i] ||
                        !param["min"].isNumeric() || !param["max"].isNumeric()) continue;
                    ranges[i] = {param["min"].asDouble(), param["max"].asDouble()};
                    cyvet::limitAxis(0, 0, ranges[i]);
                    found[i] = true;
                }
            }
        }
        capabilities_valid = actions.count("walking") && found[0] && found[1] && found[2];
        if (!capabilities_valid) throw std::runtime_error("Walking capabilities unavailable; motion disabled");
    }
    void parseLayout(const Json::Value &root) {
        layout.clear();
        const auto &motors = root.isArray() ? root : root["motors"];
        if (!motors.isArray()) return;
        std::set<std::string> names;
        for (const auto &motor : motors) {
            const auto &limb = motor.isMember("limbNo") ? motor["limbNo"] : motor["limbsNo"];
            if (!limb.isUInt() || !motor["jointNo"].isUInt() || !motor["name"].isString()) continue;
            std::string name = motor["name"].asString();
            auto key = std::make_pair(limb.asUInt(), motor["jointNo"].asUInt());
            if (name.empty() || layout.count(key) || !names.insert(name).second) {
                layout.clear(); return;
            }
            layout.emplace(key, name);
        }
    }
    void cancelQueries() {
        ++query_sequence;
        for (auto *pending : {&pending_motion, &pending_system}) {
            if (*pending) client->remove_pending_request((*pending)->id);
            pending->reset();
        }
    }
    BackendResult connectRobot() {
        started.store(true);
        if (client->getState() != Client::kDisconnected) return {true, "Connected (no automatic control acquisition)"};
        if (!client->wait_for_service(std::chrono::milliseconds(opts.connect_timeout_ms)) ||
            !client->connect(opts.lease_ms)) return error("RobotServer connection failed");
        std::string value;
        bool queried = client->queryCapabilities(value, opts.connect_timeout_ms);
        if (!queried) queried = client->queryCapabilities(value, opts.connect_timeout_ms);
        if (!queried) { client->disconnect(); return error("Capability query failed"); }
        try { parseCapabilities(parse(value)); }
        catch (const std::exception &e) { fault(e.what()); }
        if (client->queryMotorLayout(value, opts.connect_timeout_ms)) {
            try { parseLayout(parse(value)); } catch (const std::exception &e) { fault(e.what()); }
        }
        if (!client->querySystemStatus(value, opts.connect_timeout_ms)) {
            client->disconnect(); return error("Device status query failed");
        }
        updateBattery(parse(value));
        system_reply_at = Clock::now();
        observations_enabled = opts.enable_observations &&
            client->setMotionObservedEnable(true, true, opts.connect_timeout_ms);
        if (opts.enable_observations && !observations_enabled) fault("Observation enable failed");
        continuous_odom.newSession(); motion_stamp = sensor_stamp = 0;
        {
            std::lock_guard<std::mutex> lock(mutex);
            cached.connected = true; cached.control_owned = false;
        }
        return {true, "Cyvet connected; read-only until stand/ready is called"};
    }
    Json::Value velocityParams(double x, double y, double yaw) const {
        Json::Value params(Json::objectValue);
        params["lineVelocityX"] = cyvet::limitAxis(x, opts.speed_limits[speed-1][0], ranges[0]);
        params["lineVelocityY"] = cyvet::limitAxis(y * opts.lateral_sign, opts.speed_limits[speed-1][1], ranges[1]);
        params["velocity"] = cyvet::limitAxis(yaw, opts.speed_limits[speed-1][2], ranges[2]);
        return params;
    }
    BackendResult confirm(const std::string &action, bool zero, uint64_t token, bool can_cancel = true) {
        const auto end = Clock::now() + std::chrono::milliseconds(opts.action_timeout_ms);
        while (Clock::now() < end) {
            if (can_cancel && interrupted(token)) return {false, "Action canceled by stop/shutdown"};
            if (client->getState() != Client::kControlled) return {false, "Control lost during confirmation"};
            std::string value;
            if (client->queryMotionState(value, opts.rpc_timeout_ms)) {
                updateMotion(parse(value));
                BackendState snapshot = state();
                if (current_action == action && (!zero ||
                    std::abs(snapshot.vx)+std::abs(snapshot.vy)+std::abs(snapshot.vyaw) < 1e-5))
                    return {true, "Confirmed " + action};
            }
            executor->spin_some();
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait_for(lock, 20ms);
        }
        return {false, "Physical/task state confirmation timed out: " + action};
    }
    BackendResult stop(bool can_cancel = true) {
        velocity_active = false;
        if (client->getState() != Client::kControlled) return {true, "No owned motion session"};
        if (estop_latched && current_action == "emergencyStop") {
            const auto result = confirm("emergencyStop", true, generation.load(), can_cancel);
            halt_pending.store(!result.success);
            return result;
        }
        if (!client->stopAction(opts.rpc_timeout_ms)) {
            halt_pending.store(true);
            return error("Stop action failed");
        }
        const auto result = confirm("walking", true, generation.load(), can_cancel);
        halt_pending.store(!result.success);
        if (result.success) halt_escalated = false;
        return result;
    }
    BackendResult acquire(uint64_t token) {
        if (interrupted(token)) return {false, "Control request canceled"};
        if (!state().connected || !capabilities_valid) return {false, "Fresh connection and walking capabilities required"};
        if (!client->startControl(opts.action_timeout_ms)) return error("Control acquisition rejected");
        if (interrupted(token)) return {false, "Control acquired but action canceled; pending stop will release it"};
        { std::lock_guard<std::mutex> lock(mutex); cached.control_owned = true; }
        return {true, "Control acquired"};
    }
    BackendResult stand(uint64_t token) {
        if (halt_pending) return {false,"Stop is not yet confirmed; preparation refused"};
        cancelQueries();
        auto result = acquire(token);
        if (!result.success) return result;
        if (!client->startAction("walking", json(velocityParams(0, 0, 0)), opts.rpc_timeout_ms)) {
            result = error("Walking preparation rejected");
        } else result = confirm("walking", true, token);
        if (!result.success) {
            // Never enable motion after an unconfirmed action, including an ambiguous ACK timeout.
            if (!interrupted(token) && stop(false).success) releaseSdk();
            return result;
        }
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (interrupted(token)) return {false, "Preparation canceled"};
            velocity.reset(); estop_latched = false; ready.store(true);
        }
        return {true, "Walking at zero velocity confirmed; navigation ready"};
    }
    bool releaseSdk() {
        // Firmware may emit its loss-of-ownership event before the release RPC
        // ACK. Inhibit as usual, but do not report our own release as a takeover.
        releasing = true;
        const bool result = client->releaseControl();
        releasing = false;
        return result;
    }
    BackendResult release(bool can_cancel = true) {
        cancelQueries();
        const auto result = stop(can_cancel);
        if (!result.success) return result;  // Do not report release as confirmed stopping.
        if (client->getState() == Client::kControlled && !releaseSdk())
            return error("Control release failed");
        { std::lock_guard<std::mutex> lock(mutex); cached.control_owned = false; }
        return {true, "Stop confirmed and control released"};
    }
    BackendResult lie(uint64_t token) {
        cancelQueries();
        if (interrupted(token)) { stop(false); return {false, "Laying superseded by stop"}; }
        if (!actions.count("laying")) return {false, "Device does not advertise laying"};
        auto result = acquire(token);
        if (!result.success) return result;
        if (!client->startAction("laying", "{}", opts.rpc_timeout_ms)) result = error("Laying rejected");
        else result = confirm("laying", false, token);
        if (!result.success) {
            if (!interrupted(token) && stop(false).success) releaseSdk();
            return result;
        }
        if (!releaseSdk()) return error("Laying confirmed but release failed");
        return {true, "Laying confirmed and control released"};
    }
    BackendResult estop(uint64_t token) {
        cancelQueries(); estop_latched = true; velocity_active = false;
        if (client->getState() != Client::kControlled) return {false, "No control: local motion inhibited; remote emergency stop not sent"};
        if (!client->emergencyStop(opts.rpc_timeout_ms)) {
            halt_pending.store(true);
            return error("Emergency stop failed; local inhibition remains");
        }
        const auto result = confirm("emergencyStop", true, token, false);
        halt_pending.store(!result.success);
        if (!result.success) return result;
        return {true, "Emergency stop confirmed; explicit stand/ready required"};
    }
    void poll(const std::string &method, std::optional<Pending> &pending) {
        const auto now = Clock::now();
        if (pending) {
            if (now > pending->deadline) { client->remove_pending_request(pending->id); pending.reset(); }
            else return;
        }
        const auto sequence = ++query_sequence;
        auto future = client->async_call(Rpc("robotAppService", method, Json::Value(Json::nullValue),
            client->client_id(), opts.device_id, 0),
            [this, method, sequence](Client::SharedFuture result) {
                auto &slot = method == "queryMotionState" ? pending_motion : pending_system;
                if (!slot || slot->sequence != sequence) return;
                slot.reset();
                try {
                    auto response = result.get();
                    if (response->code != 0 || response->device_id != opts.device_id)
                        throw std::runtime_error("Read-only RPC returned error or wrong identity");
                    const auto value = parse(response->payload);
                    if (!value.get("result", false).asBool()) throw std::runtime_error("Read-only query rejected");
                    const auto &params = value.isMember("params") ? value["params"] : Json::Value(Json::objectValue);
                    if (method == "queryMotionState") updateMotion(params);
                    else { updateBattery(params); system_reply_at = Clock::now(); }
                } catch (const std::exception &e) { fault(e.what()); }
            });
        pending = Pending{future.request_id, sequence, now + std::chrono::milliseconds(opts.rpc_timeout_ms)};
    }
    void dispatchVelocity() {
        if (!ready || client->getState() != Client::kControlled) { velocity_active = false; return; }
        std::optional<Velocity> command;
        { std::lock_guard<std::mutex> lock(mutex); command = velocity; }
        const auto now = Clock::now();
        if (!command || command->generation != generation.load() ||
            now-command->at > std::chrono::milliseconds(opts.cmd_vel_timeout_ms)) {
            if (velocity_active) {
                velocity_active = false;
                auto result = stop();
                if (!result.success) { inhibit(); fault(result.message); }
            }
            return;
        }
        if (now < next_velocity) return;
        next_velocity = now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1 / opts.cmd_vel_rate_hz));
        const auto params = velocityParams(command->x, command->y, command->yaw);
        // Mark active before RPC; a timed-out request may still have taken effect.
        velocity_active = true;
        if (!client->setActionParams(json(params), opts.rpc_timeout_ms)) {
            inhibit(); error("Velocity rejected or timed out");
            const auto result = stop();
            if (!result.success) fault(result.message);
        }
    }
    void recoverStop() {
        if (!halt_pending) return;
        if (client->getState() != Client::kControlled) {
            halt_pending.store(false);
            halt_escalated = false;
            return;  // No ability to confirm physical stop after ownership loss.
        }
        if (Clock::now() < next_halt) return;
        if (!halt_escalated) {
            halt_escalated = true;
            estop_latched = true;
            if (!client->emergencyStop(opts.rpc_timeout_ms))
                error("Stop fallback emergencyStop failed; retrying stop");
        }
        // An ambiguous RPC acknowledgement must not keep renewing an unsafe
        // action forever. Read the effective state after the emergency fallback.
        std::string value;
        if (client->queryMotionState(value,opts.rpc_timeout_ms)) {
            updateMotion(parse(value));
            const auto snapshot = state();
            if ((current_action == "walking" || current_action == "emergencyStop") &&
                std::abs(snapshot.vx)+std::abs(snapshot.vy)+std::abs(snapshot.vyaw) < 1e-5) {
                halt_pending.store(false);
                halt_escalated = false;
                return;
            }
        }
        auto result = stop(false);
        if (!result.success) fault(result.message);
        next_halt = Clock::now() + 200ms;
    }
    BackendState state() const {
        std::lock_guard<std::mutex> lock(mutex);
        auto result = cached;
        if (Clock::now()-battery_at > std::chrono::milliseconds(opts.telemetry_timeout_ms))
            result.battery_percent = -1;
        return result;
    }
    void run() {
        while (!quitting) {
            try {
                std::shared_ptr<Job> job;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    cv.wait_for(lock, 5ms, [this] { return quitting || !urgent.empty() || !normal.empty(); });
                    auto &queue = urgent.empty() ? normal : urgent;
                    if (!queue.empty()) { job = queue.front(); queue.pop_front(); }
                }
                if (job) {
                    try {
                        job->result.set_value(!job->priority && interrupted(job->generation) ?
                            BackendResult{false, "Superseded by stop or shutdown"} : job->action(job->generation));
                    } catch (const std::exception &e) { inhibit(); fault(e.what()); job->result.set_value({false, e.what()}); }
                    // Acquisition includes a firmware master-switch wait. Refresh
                    // liveness before evaluating the idle asynchronous query watchdog.
                    if (client->getState() != Client::kDisconnected) {
                        std::string value;
                        if (client->queryMotionState(value,opts.rpc_timeout_ms)) updateMotion(parse(value));
                    }
                }
                if (quitting) break;
                executor->spin_some();
                if (client->getState() == Client::kDisconnected) {
                    if (started && Clock::now() >= next_connect) {
                        next_connect = Clock::now() + std::chrono::milliseconds(opts.reconnect_interval_ms);
                        connectRobot();
                        next_connect = Clock::now() + std::chrono::milliseconds(opts.reconnect_interval_ms);
                    }
                    continue;
                }
                const auto now = Clock::now();
                if (ready && now-motion_reply_at > std::chrono::milliseconds(opts.telemetry_timeout_ms)) {
                    inhibit();
                    fault("Motion state stale; explicit stand/ready required");
                    auto result=stop();
                    if (!result.success) fault(result.message);
                }
                dispatchVelocity();
                recoverStop();
                if (now >= next_motion) { next_motion = now+100ms; poll("queryMotionState", pending_motion); }
                if (now >= next_system) { next_system = now+1s; poll("getSystemStatus", pending_system); }
                if (now-std::max(system_reply_at, motion_reply_at) > std::chrono::milliseconds(opts.telemetry_timeout_ms)) {
                    inhibit(); cancelQueries(); fault("Robot RPC stale; reconnecting without control");
                    if (client->getState() == Client::kControlled) {
                        if (!stop().success) client->emergencyStop(opts.rpc_timeout_ms);
                    }
                    client->disconnect(); observations_enabled = false;
                    std::lock_guard<std::mutex> lock(mutex);
                    cached = BackendState{};
                }
                StateCallback callback;
                { std::lock_guard<std::mutex> lock(mutex); callback = state_callback; }
                if (callback) callback(state());
            } catch (const std::exception &e) {
                inhibit(); fault(e.what());
                std::unique_lock<std::mutex> lock(mutex); cv.wait_for(lock, 100ms);
            }
        }
        // Cleanup while the robot context is still alive (including SIGINT).
        cancelQueries();
        if (client->getState() == Client::kControlled) {
            auto result = release(false);
            if (!result.success) { fault(result.message); client->emergencyStop(opts.rpc_timeout_ms); }
        }
        if (observations_enabled) client->setMotionObservedEnable(false, false, opts.rpc_timeout_ms);
        client->disconnect();
        std::lock_guard<std::mutex> lock(mutex);
        for (auto *queue : {&urgent, &normal}) {
            for (auto &job : *queue) job->result.set_value({false, "Backend shutdown"});
            queue->clear();
        }
    }
    void onMotion(const uniubi::msg::MotionObserved &sample) {
        if (sample.timestamp <= motion_stamp) return;
        motion_stamp = sample.timestamp;
        if (std::isfinite(sample.power.power) && sample.power.power >= 0 && sample.power.power <= 100 &&
            std::isfinite(sample.power.charge_voltage) && sample.power.charge_voltage > 0) {
            std::lock_guard<std::mutex> lock(mutex);
            cached.battery_percent = sample.power.power; battery_at = Clock::now();
        }
        const auto &a = sample.imu.accel; const auto &g = sample.imu.gyro;
        const auto &q = sample.imu.quaternion;
        if (!a.error && !g.error && finite({a.x,a.y,a.z,g.x,g.y,g.z})) {
            BackendImu imu{a.x,a.y,a.z,g.x,g.y,g.z};
            double norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
            if (!q.error && std::isfinite(norm) && norm > 1e-6) {
                imu.qx=q.x/norm; imu.qy=q.y/norm; imu.qz=q.z/norm; imu.qw=q.w/norm;
            } else imu.qw = 0;  // Node marks orientation as unavailable.
            ImuCallback callback;
            { std::lock_guard<std::mutex> lock(mutex); callback = imu_callback; }
            if (callback) callback(imu);
        }
        if (sample.motor_num <= 0 || sample.motor_num > static_cast<int>(sample.motor.size()) ||
            static_cast<size_t>(sample.motor_num) != layout.size()) return;
        BackendJointState joints;
        for (int i=0; i<sample.motor_num; ++i) {
            const auto &motor=sample.motor[static_cast<size_t>(i)];
            auto found=layout.find({motor.header.limbs_no,motor.header.joint_no});
            if (found==layout.end() || !motor.online || motor.error ||
                !finite({motor.position,motor.velocity,motor.torque})) continue;
            joints.names.push_back(found->second); joints.positions.push_back(motor.position);
            joints.velocities.push_back(motor.velocity); joints.efforts.push_back(motor.torque);
        }
        if (joints.names.size()!=layout.size() ||
            std::set<std::string>(joints.names.begin(),joints.names.end()).size()!=layout.size()) return;
        JointCallback callback;
        { std::lock_guard<std::mutex> lock(mutex); callback = joint_callback; }
        if (callback) callback(joints);
    }
    void onSensor(const uniubi::msg::SensorObserved &sample) {
        if (sample.timestamp <= sensor_stamp) return;
        sensor_stamp=sample.timestamp;
        const auto &o=sample.odom;
        if (!o.valid || !finite({o.position[0],o.position[1],o.yaw,o.velocity[0],o.velocity[1],o.yaw_speed})) return;
        auto pose=continuous_odom.update(o.epoch,{o.position[0],o.position[1],o.yaw});
        BackendOdometry odom;
        odom.px=pose.x; odom.py=pose.y;
        odom.qz=std::sin(pose.yaw/2); odom.qw=std::cos(pose.yaw/2);
        odom.vx=o.velocity[0]; odom.vy=o.velocity[1]; odom.wz=o.yaw_speed;
        OdometryCallback callback;
        { std::lock_guard<std::mutex> lock(mutex); callback = odom_callback; }
        if (callback) callback(odom);
    }
};

CyvetBackend::CyvetBackend(CyvetOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}
CyvetBackend::~CyvetBackend() = default;
BackendResult CyvetBackend::connect() { return impl_->submit([this](uint64_t) { return impl_->connectRobot(); }); }
BackendResult CyvetBackend::disconnect() {
    return impl_->submitStop([this](uint64_t) {
        impl_->started.store(false);
        auto result = impl_->release(false);
        if (!result.success) return result;
        if (impl_->observations_enabled) impl_->client->setMotionObservedEnable(false,false,impl_->opts.rpc_timeout_ms);
        impl_->observations_enabled=false; impl_->client->disconnect();
        { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->cached=BackendState{}; }
        return BackendResult{true,"Disconnected"};
    });
}
BackendResult CyvetBackend::takeControl() { return impl_->submit([this](uint64_t token) { return impl_->acquire(token); }); }
BackendResult CyvetBackend::stand() {
    return impl_->submit([this](uint64_t token) { return impl_->stand(token); },false,true);
}
BackendResult CyvetBackend::lie() {
    return impl_->submitStop([this](uint64_t token) { return impl_->lie(token); });
}
BackendResult CyvetBackend::releaseControl() {
    return impl_->submitStop([this](uint64_t) { return impl_->release(false); });
}
BackendResult CyvetBackend::softEstop(bool enabled) {
    if (!enabled) return stand();
    return impl_->submitStop([this](uint64_t token) { return impl_->estop(token); });
}
BackendResult CyvetBackend::setMode(int mode) {
    if (mode != 0) return {false,"Cyvet supports only WALK/0; legacy gaits are not equivalent"};
    return {true,"WALK selected; explicit stand/ready required to enable motion"};
}
BackendResult CyvetBackend::setSpeed(int level) {
    if (level<1 || level>3) return {false,"Speed must be 1, 2 or 3"};
    return impl_->submit([this,level](uint64_t) {
        impl_->speed=level; return BackendResult{true,"Local speed limit changed; vendor profile remains unchanged"};
    });
}
BackendResult CyvetBackend::move(double x,double y,double yaw) {
    if (!finite({x,y,yaw})) return {false,"Non-finite velocity rejected"};
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->ready || !impl_->cached.control_owned || !impl_->cached.connected) return {false,"Explicit stand/ready required"};
    impl_->velocity=Impl::Velocity{x,y,yaw,Clock::now(),impl_->generation.load()};
    return {true,"Latest velocity accepted for dispatch"};
}
BackendState CyvetBackend::state() const { return impl_->state(); }
std::string CyvetBackend::diagnostics() const {
    const auto state=impl_->state();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Json::Value value;
    value["device_id"]=impl_->opts.device_id; value["connected"]=state.connected;
    value["control_owned"]=state.control_owned; value["navigation_ready"]=impl_->ready.load();
    value["stop_pending"]=impl_->halt_pending.load();
    value["action"]=impl_->current_action;
    value["vx"]=state.vx; value["vy"]=state.vy; value["yaw_rate"]=state.vyaw;
    value["last_error"]=impl_->last_fault; value["battery_valid"]=state.battery_percent>=0;
    value["command_cached"]=impl_->velocity.has_value();
    if (impl_->velocity) {
        value["command_age_ms"]=static_cast<Json::Int64>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-impl_->velocity->at).count());
        value["command_current"]=impl_->velocity->generation==impl_->generation.load();
    }
    return json(value);
}
#define CYVET_CALLBACK(Method, Type, Member) \
    void CyvetBackend::Method(Type callback) { \
        std::lock_guard<std::mutex> lock(impl_->mutex); impl_->Member=std::move(callback); \
    }
CYVET_CALLBACK(setStateCallback, StateCallback, state_callback)
CYVET_CALLBACK(setImuCallback, ImuCallback, imu_callback)
CYVET_CALLBACK(setOdometryCallback, OdometryCallback, odom_callback)
CYVET_CALLBACK(setJointCallback, JointCallback, joint_callback)
CYVET_CALLBACK(setFaultCallback, FaultCallback, fault_callback)
#undef CYVET_CALLBACK
}  // namespace nav_bridge
