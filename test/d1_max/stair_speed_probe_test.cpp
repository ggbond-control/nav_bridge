// Offline checks only: SDK methods are substituted, never connected to a robot.
#include <gtest/gtest.h>

#define D1_MAX_STAIR_PROBE_NO_MAIN
#include "stair_speed_probe.cpp"

namespace probe_mock {
std::shared_ptr<robot_sdk::IDataCallback> callback;
robot_sdk::MotionStatus posture = robot_sdk::MotionStatus::MOTION_STATUS_WALK;
robot_sdk::CtrlSource control = robot_sdk::CtrlSource::CTRL_SOURCE_APP;
std::vector<std::array<float, 3>> moves;
std::vector<std::string> commands;
bool connected = false, lose_control_on_move = false, fail_nonzero = false, interrupt_on_move = false;
int speed_calls = 0, stand_calls = 0;

void report(float lateral = 0, float forward = 0, float yaw = 0) {
    robot_sdk::RobotState state{};
    state.motion_status = posture;
    state.control_source = control;
    state.speed_level = robot_sdk::SpeedLevel::SPEED_LEVEL_HIGH;
    callback->OnRobotStateData(state);
    robot_sdk::SpeedData speed{forward * 0.7F, lateral * 0.3F, yaw * 0.8F};
    callback->OnSpeedData(speed);
    robot_sdk::MotionData motion{};
    motion.v_body[0] = speed.x;
    motion.v_body[1] = speed.y;
    motion.omega_body[2] = speed.yaw;
    motion.time_stamp = 123456;
    callback->OnMcData(motion);
}
}  // namespace probe_mock

namespace robot_sdk {
void SDKClient::SetDataCallback(std::shared_ptr<IDataCallback> callback) { probe_mock::callback = callback; }
std::error_code SDKClient::Connect(std::string, std::string, bool, ConnectHandler) {
    probe_mock::connected = true;
    return {};
}
bool SDKClient::IsConnected() const { return probe_mock::connected; }
std::error_code SDKClient::Disconnect(bool, DisConnectHandler) { probe_mock::connected = false; return {}; }
std::error_code SDKClient::SetSpeedReportConfig(bool, uint32_t, int, WriteHandler) { return {}; }
std::error_code SDKClient::SetMcConfig(bool, int, WriteHandler) { probe_mock::report(); return {}; }
std::error_code SDKClient::TakeControl(int, WriteHandler) {
    probe_mock::commands.push_back("TakeControl");
    probe_mock::control = CtrlSource::CTRL_SOURCE_SDK;
    probe_mock::report();
    return {};
}
std::error_code SDKClient::ReleaseControl(int, WriteHandler) {
    probe_mock::commands.push_back("ReleaseControl");
    probe_mock::control = CtrlSource::CTRL_SOURCE_APP;
    return {};
}
std::error_code SDKClient::Stair(int, WriteHandler) {
    probe_mock::commands.push_back("Stair");
    probe_mock::posture = MotionStatus::MOTION_STATUS_STAIR;
    probe_mock::report();
    return {};
}
std::error_code SDKClient::Move(float lateral, float forward, float yaw, int, WriteHandler) {
    probe_mock::moves.push_back({lateral, forward, yaw});
    const bool nonzero = lateral != 0 || forward != 0 || yaw != 0;
    if (nonzero && probe_mock::fail_nonzero) return std::make_error_code(std::errc::io_error);
    probe_mock::report(lateral, forward, yaw);
    if (nonzero && probe_mock::lose_control_on_move) {
        probe_mock::callback->OnControlLost(ControlLostInfo{});
    }
    if (nonzero && probe_mock::interrupt_on_move) stair_probe::signalHandler(SIGINT);
    return {};
}
std::error_code SDKClient::SetSpeed(int, int, WriteHandler) { ++probe_mock::speed_calls; return {}; }
std::error_code SDKClient::StandUp(int, WriteHandler) { ++probe_mock::stand_calls; return {}; }
}  // namespace robot_sdk

namespace stair_probe {
class StairProbeTest : public testing::Test {
protected:
    void SetUp() override {
        interrupted = 0;
        probe_mock::callback.reset();
        probe_mock::moves.clear();
        probe_mock::commands.clear();
        probe_mock::posture = MotionStatus::MOTION_STATUS_WALK;
        probe_mock::control = CtrlSource::CTRL_SOURCE_APP;
        probe_mock::connected = false;
        probe_mock::lose_control_on_move = probe_mock::fail_nonzero = false;
        probe_mock::interrupt_on_move = false;
        probe_mock::speed_calls = probe_mock::stand_calls = 0;
    }
    Options options() {
        Options value;
        value.host = "127.0.0.1";
        value.execute = true;
        value.settle = value.duration = 0.2;
        value.samples = {{0.2, -0.1, 0.3}};
        return value;
    }
    void expectStoppedAndReleased() {
        ASSERT_FALSE(probe_mock::moves.empty());
        EXPECT_EQ(probe_mock::moves.back(), (std::array<float, 3>{0, 0, 0}));
        ASSERT_FALSE(probe_mock::commands.empty());
        EXPECT_EQ(probe_mock::commands.back(), "ReleaseControl");
        EXPECT_FALSE(probe_mock::connected);
        EXPECT_EQ(probe_mock::speed_calls, 0);
        EXPECT_EQ(probe_mock::stand_calls, 0);
    }
};

TEST_F(StairProbeTest, PreviewDoesNotConstructSessionOrConnect) {
    char name[] = "probe", option[] = "--sample", sample[] = "0.1,0,0";
    char *argv[] = {name, option, sample};
    EXPECT_EQ(run(3, argv), 0);
    EXPECT_EQ(probe_mock::callback, nullptr);
    EXPECT_TRUE(probe_mock::commands.empty());
}

TEST_F(StairProbeTest, RejectsInvalidOrOutOfRangeInputs) {
    for (const auto *text : {"", "0,0", "0,0,0,", "1.1,0,0", "nan,0,0", "0,inf,0", "0,0,1junk"}) {
        SCOPED_TRACE(text);
        EXPECT_THROW(parseCommand(text), std::exception);
    }
}

TEST_F(StairProbeTest, MeasuresDirectPercentagesAndBothFeedbackSources) {
    std::ostringstream output;
    ASSERT_NO_THROW(execute(options(), output));
    EXPECT_EQ(probe_mock::commands, (std::vector<std::string>{"TakeControl", "Stair", "ReleaseControl"}));
    EXPECT_NE(std::find(probe_mock::moves.begin(), probe_mock::moves.end(),
                        std::array<float, 3>{-0.1F, 0.2F, 0.3F}), probe_mock::moves.end());
    EXPECT_NE(output.str().find(",measure,0.2,-0.1,0.3,speed,"), std::string::npos);
    EXPECT_NE(output.str().find(",measure,0.2,-0.1,0.3,mc_body,"), std::string::npos);
    expectStoppedAndReleased();
}

TEST_F(StairProbeTest, ControlLossAbortsAndSendsZero) {
    probe_mock::lose_control_on_move = true;
    std::ostringstream output;
    EXPECT_THROW(execute(options(), output), std::runtime_error);
    expectStoppedAndReleased();
}

TEST_F(StairProbeTest, SendFailureAbortsAndSendsZero) {
    probe_mock::fail_nonzero = true;
    std::ostringstream output;
    EXPECT_THROW(execute(options(), output), std::runtime_error);
    expectStoppedAndReleased();
}

TEST_F(StairProbeTest, InterruptAbortsAndSendsZero) {
    probe_mock::interrupt_on_move = true;
    std::ostringstream output;
    EXPECT_THROW(execute(options(), output), std::runtime_error);
    expectStoppedAndReleased();
}

TEST_F(StairProbeTest, WatchdogRejectsStaleFeedbackAndWrongPosture) {
    Snapshot s;
    s.state_at = s.speed_at = s.mc_at = Clock::now();
    s.state.motion_status = MotionStatus::MOTION_STATUS_STAIR;
    s.state.control_source = CtrlSource::CTRL_SOURCE_SDK;
    ASSERT_NO_THROW(healthy(s, true, true));
    s.mc_at -= std::chrono::seconds(1);
    EXPECT_THROW(healthy(s, true, true), std::runtime_error);
    s.mc_at = Clock::now();
    s.state.motion_status = MotionStatus::MOTION_STATUS_GAIT;
    EXPECT_THROW(healthy(s, true, true), std::runtime_error);
    s.state.motion_status = MotionStatus::MOTION_STATUS_STAIR;
    s.speed[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(healthy(s, true, true), std::runtime_error);
}
}  // namespace stair_probe
