#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "nav_bridge/d1_max/d1_max_backend.hpp"
#include "robot_sdk/sdk_client.hpp"

// Exercise the real backend with SDK command substitutes; no robot connection
// or network commands are needed to check posture selection and velocity units.
namespace sdk_gait_test {
std::weak_ptr<robot_sdk::IDataCallback> callback;
int gait_calls = 0;
int stair_calls = 0;
int stand_calls = 0;
std::vector<int> speeds;
std::vector<std::array<float, 3>> moves;
std::error_code speed_error;
std::vector<robot_sdk::MotionStatus> special_postures;
std::error_code posture_error;
bool acknowledge_posture = true;
robot_sdk::MotionStatus current_posture = robot_sdk::MotionStatus::MOTION_STATUS_WALK;
robot_sdk::SpeedLevel current_speed = robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM;
bool acknowledge_speed = true;
int speed_feedback_override = 0;
robot_sdk::MotionStatus stand_feedback = robot_sdk::MotionStatus::MOTION_STATUS_WALK;
std::error_code stand_error;
std::vector<std::string> commands;

void posture(robot_sdk::MotionStatus status) {
    current_posture = status;
    robot_sdk::RobotState data{};
    data.motion_status = status;
    data.speed_level = current_speed;
    data.control_source = robot_sdk::CtrlSource::CTRL_SOURCE_SDK;
    callback.lock()->OnRobotStateData(data);
}

std::error_code selectPosture(robot_sdk::MotionStatus status) {
    special_postures.push_back(status);
    if (posture_error) return posture_error;
    if (acknowledge_posture) posture(status);
    return {};
}
}  // namespace sdk_gait_test

namespace robot_sdk {
bool SDKClient::IsConnected() const { return true; }
std::error_code SDKClient::Disconnect(bool, DisConnectHandler) { return {}; }
void SDKClient::SetDataCallback(std::shared_ptr<IDataCallback> callback) {
    sdk_gait_test::callback = callback;
}
std::error_code SDKClient::TakeControl(int, WriteHandler) { return {}; }
std::error_code SDKClient::Move(float lateral, float forward, float yaw, int, WriteHandler) {
    sdk_gait_test::commands.push_back("Move");
    sdk_gait_test::moves.push_back({lateral, forward, yaw});
    return {};
}
std::error_code SDKClient::SetSpeed(int speed, int, WriteHandler) {
    sdk_gait_test::commands.push_back("SetSpeed");
    sdk_gait_test::speeds.push_back(speed);
    if (!sdk_gait_test::speed_error && sdk_gait_test::acknowledge_speed) {
        sdk_gait_test::current_speed = static_cast<SpeedLevel>(
            sdk_gait_test::speed_feedback_override ? sdk_gait_test::speed_feedback_override : speed);
        sdk_gait_test::posture(sdk_gait_test::current_posture);
    }
    return sdk_gait_test::speed_error;
}
std::error_code SDKClient::Gait(int, WriteHandler) {
    sdk_gait_test::commands.push_back("Gait");
    ++sdk_gait_test::gait_calls;
    sdk_gait_test::posture(MotionStatus::MOTION_STATUS_GAIT);
    return {};
}
std::error_code SDKClient::Stair(int, WriteHandler) {
    ++sdk_gait_test::stair_calls;
    sdk_gait_test::posture(MotionStatus::MOTION_STATUS_STAIR);
    return {};
}
std::error_code SDKClient::StandUp(int, WriteHandler) {
    sdk_gait_test::commands.push_back("StandUp");
    ++sdk_gait_test::stand_calls;
    if (sdk_gait_test::stand_error) return sdk_gait_test::stand_error;
    sdk_gait_test::posture(sdk_gait_test::stand_feedback);
    return {};
}
std::error_code SDKClient::CrawlWalk(int, WriteHandler) {
    return sdk_gait_test::selectPosture(MotionStatus::MOTION_STATUS_CRAWL_WALK);
}
std::error_code SDKClient::Climb(int, WriteHandler) {
    return sdk_gait_test::selectPosture(MotionStatus::MOTION_STATUS_CLIMB);
}
std::error_code SDKClient::Slim(int, WriteHandler) {
    return sdk_gait_test::selectPosture(MotionStatus::MOTION_STATUS_SLIM);
}
std::error_code SDKClient::DSB(int, WriteHandler) {
    return sdk_gait_test::selectPosture(MotionStatus::MOTION_STATUS_DSB);
}
}  // namespace robot_sdk

namespace nav_bridge {
struct SpecialPostureMapping {
    int gait;
    robot_sdk::MotionStatus status;
};
constexpr SpecialPostureMapping special_mappings[] = {
    {d1_max_gait::CRAWL_WALK, robot_sdk::MotionStatus::MOTION_STATUS_CRAWL_WALK},
    {d1_max_gait::CLIMB, robot_sdk::MotionStatus::MOTION_STATUS_CLIMB},
    {d1_max_gait::SLIM, robot_sdk::MotionStatus::MOTION_STATUS_SLIM},
    {d1_max_gait::DSB, robot_sdk::MotionStatus::MOTION_STATUS_DSB},
};

class D1MaxGaitTest : public testing::Test {
protected:
    D1MaxBackend backend{"127.0.0.1", 8082, false, 100, 100, false};

    void SetUp() override {
        sdk_gait_test::gait_calls = 0;
        sdk_gait_test::stair_calls = 0;
        sdk_gait_test::stand_calls = 0;
        sdk_gait_test::speeds.clear();
        sdk_gait_test::moves.clear();
        sdk_gait_test::speed_error = {};
        sdk_gait_test::special_postures.clear();
        sdk_gait_test::posture_error = {};
        sdk_gait_test::acknowledge_posture = true;
        sdk_gait_test::current_speed = robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM;
        sdk_gait_test::acknowledge_speed = true;
        sdk_gait_test::speed_feedback_override = 0;
        sdk_gait_test::stand_feedback = robot_sdk::MotionStatus::MOTION_STATUS_WALK;
        sdk_gait_test::stand_error = {};
        sdk_gait_test::commands.clear();
        sdk_gait_test::posture(robot_sdk::MotionStatus::MOTION_STATUS_WALK);
    }

    void expectMove(double forward_scale) {
        ASSERT_TRUE(backend.move(0.6, -0.1, 0.3).success);
        ASSERT_FALSE(sdk_gait_test::moves.empty());
        const auto &move = sdk_gait_test::moves.back();
        EXPECT_FLOAT_EQ(move[0], -0.2F);
        EXPECT_FLOAT_EQ(move[1], static_cast<float>(0.6 / forward_scale));
        EXPECT_FLOAT_EQ(move[2], 0.2F);
    }
};

TEST_F(D1MaxGaitTest, NormalGaitsAlreadyInWalkOnlyAdjustSpeed) {
    struct Mapping { int gait; int speed; };
    for (const auto mapping : {Mapping{0, 1}, Mapping{3, 3}, Mapping{34, 2}}) {
        SCOPED_TRACE(mapping.gait);
        sdk_gait_test::speeds.clear();
        sdk_gait_test::moves.clear();
        ASSERT_TRUE(backend.setGait(mapping.gait).success);
        EXPECT_EQ(sdk_gait_test::gait_calls, 0);
        EXPECT_EQ(sdk_gait_test::stair_calls, 0);
        EXPECT_EQ(sdk_gait_test::stand_calls, 0);
        EXPECT_EQ(sdk_gait_test::speeds, std::vector<int>{mapping.speed});
        EXPECT_EQ(backend.state().mode, mapping.gait);
        ASSERT_EQ(sdk_gait_test::moves.size(), 1U);
        EXPECT_EQ(sdk_gait_test::moves.front(), (std::array<float, 3>{0, 0, 0}));
        expectMove(mapping.speed);
    }
}

TEST_F(D1MaxGaitTest, LWalkSelectsDedicatedGaitAndMediumSpeed) {
    ASSERT_TRUE(backend.setSpeed(3).success);
    sdk_gait_test::speeds.clear();
    sdk_gait_test::commands.clear();
    ASSERT_TRUE(backend.setGait(32).success);
    EXPECT_EQ(sdk_gait_test::gait_calls, 1);
    EXPECT_EQ(sdk_gait_test::stair_calls, 0);
    EXPECT_EQ(sdk_gait_test::commands, (std::vector<std::string>{"Move", "Gait", "SetSpeed"}));
    EXPECT_EQ(sdk_gait_test::speeds, std::vector<int>{2});
    EXPECT_EQ(sdk_gait_test::current_speed, robot_sdk::SpeedLevel::SPEED_LEVEL_MEDIUM);
    expectMove(2.0);
}

TEST_F(D1MaxGaitTest, LWalkNormalizationFollowsSubsequentSpeedSetting) {
    ASSERT_TRUE(backend.setGait(32).success);
    ASSERT_TRUE(backend.setSpeed(3).success);
    expectMove(3.0);
    ASSERT_TRUE(backend.setSpeed(1).success);
    expectMove(1.0);
}

TEST_F(D1MaxGaitTest, MountainSelectsGaitAndHighSpeed) {
    ASSERT_TRUE(backend.setGait(33).success);
    EXPECT_EQ(sdk_gait_test::gait_calls, 1);
    EXPECT_EQ(sdk_gait_test::stair_calls, 0);
    EXPECT_EQ(sdk_gait_test::speeds, std::vector<int>{3});
    EXPECT_EQ(backend.state().mode, 33);
    expectMove(3.0);
}

TEST_F(D1MaxGaitTest, StairMappingsKeepStairAndFixedNormalization) {
    ASSERT_TRUE(backend.setSpeed(3).success);
    sdk_gait_test::speeds.clear();
    int expected_calls = 0;
    for (int gait : {6, 7, 8, 36}) {
        SCOPED_TRACE(gait);
        ASSERT_TRUE(backend.setGait(gait).success);
        EXPECT_EQ(sdk_gait_test::gait_calls, 0);
        EXPECT_EQ(sdk_gait_test::stair_calls, ++expected_calls);
        EXPECT_TRUE(sdk_gait_test::speeds.empty());
        expectMove(1.0);
    }
}

TEST_F(D1MaxGaitTest, StandFromLieDownPreparesMountainGaitAndHighSpeed) {
    sdk_gait_test::posture(robot_sdk::MotionStatus::MOTION_STATUS_LIE_DOWN);
    ASSERT_TRUE(backend.stand().success);
    EXPECT_EQ(sdk_gait_test::stand_calls, 1);
    EXPECT_EQ(sdk_gait_test::gait_calls, 1);
    EXPECT_EQ(sdk_gait_test::speeds, std::vector<int>{3});
    EXPECT_EQ(backend.state().mode, 33);
    expectMove(3.0);
}

TEST_F(D1MaxGaitTest, FailedSpeedChangePreservesPreviousGaitAndNormalization) {
    ASSERT_TRUE(backend.setGait(0).success);
    sdk_gait_test::speed_error = std::make_error_code(std::errc::io_error);
    for (int gait : {32, 33}) {
        SCOPED_TRACE(gait);
        EXPECT_FALSE(backend.setGait(gait).success);
        EXPECT_EQ(backend.state().mode, 0);
        expectMove(1.0);
    }
    EXPECT_EQ(sdk_gait_test::gait_calls, 2);
}

TEST_F(D1MaxGaitTest, SpecialPosturesSelectSdkCommandAndAllowMoveWithoutChangingSpeed) {
    ASSERT_TRUE(backend.setSpeed(3).success);
    sdk_gait_test::speeds.clear();
    for (const auto &mapping : special_mappings) {
        SCOPED_TRACE(mapping.gait);
        sdk_gait_test::special_postures.clear();
        sdk_gait_test::moves.clear();
        ASSERT_TRUE(backend.setGait(mapping.gait).success);
        EXPECT_EQ(sdk_gait_test::special_postures,
                  std::vector<robot_sdk::MotionStatus>{mapping.status});
        EXPECT_EQ(backend.state().mode, mapping.gait);
        EXPECT_TRUE(backend.velocityCommandAllowed());
        EXPECT_EQ(backend.bodyHeightState(), mapping.gait == d1_max_gait::CRAWL_WALK ? -1 : 0);
        EXPECT_TRUE(sdk_gait_test::speeds.empty());
        EXPECT_EQ(sdk_gait_test::current_speed, robot_sdk::SpeedLevel::SPEED_LEVEL_HIGH);
        EXPECT_EQ(sdk_gait_test::gait_calls, 0);
        EXPECT_EQ(sdk_gait_test::stair_calls, 0);
        ASSERT_EQ(sdk_gait_test::moves.size(), 1U);
        EXPECT_EQ(sdk_gait_test::moves.front(), (std::array<float, 3>{0, 0, 0}));
        expectMove(1.0);
    }
}

TEST_F(D1MaxGaitTest, SpecialPosturesCanReturnToMountain) {
    for (const auto &mapping : special_mappings) {
        SCOPED_TRACE(mapping.gait);
        ASSERT_TRUE(backend.setGait(mapping.gait).success);
        sdk_gait_test::moves.clear();
        ASSERT_TRUE(backend.setGait(33).success);
        EXPECT_EQ(backend.state().mode, 33);
        EXPECT_EQ(sdk_gait_test::speeds.back(), 3);
        ASSERT_EQ(sdk_gait_test::moves.size(), 1U);
        EXPECT_EQ(sdk_gait_test::moves.front(), (std::array<float, 3>{0, 0, 0}));
    }
    EXPECT_EQ(sdk_gait_test::gait_calls, 4);
}

TEST_F(D1MaxGaitTest, SpecialPostureSdkErrorsPreservePreviousGait) {
    ASSERT_TRUE(backend.setGait(0).success);
    sdk_gait_test::posture_error = std::make_error_code(std::errc::io_error);
    for (const auto &mapping : special_mappings) {
        SCOPED_TRACE(mapping.gait);
        EXPECT_FALSE(backend.setGait(mapping.gait).success);
        EXPECT_EQ(backend.state().mode, 0);
        expectMove(1.0);
    }
}

TEST_F(D1MaxGaitTest, SpecialPostureSendSuccessRequiresMatchingFeedback) {
    ASSERT_TRUE(backend.setGait(0).success);
    sdk_gait_test::acknowledge_posture = false;
    for (const auto &mapping : special_mappings) {
        SCOPED_TRACE(mapping.gait);
        const auto result = backend.setGait(mapping.gait);
        EXPECT_FALSE(result.success);
        EXPECT_NE(result.message.find("Timeout waiting"), std::string::npos);
        EXPECT_EQ(backend.state().mode, 0);
        expectMove(1.0);
    }
}

TEST_F(D1MaxGaitTest, MountainToRunStopsAndLeavesGaitBeforeSettingHighSpeed) {
    ASSERT_TRUE(backend.setGait(33).success);
    sdk_gait_test::commands.clear();
    ASSERT_TRUE(backend.setGait(3).success);
    EXPECT_EQ(sdk_gait_test::commands, (std::vector<std::string>{"Move", "StandUp", "SetSpeed"}));
    EXPECT_EQ(sdk_gait_test::stand_calls, 1);
    EXPECT_EQ(sdk_gait_test::current_posture, robot_sdk::MotionStatus::MOTION_STATUS_WALK);
    EXPECT_EQ(sdk_gait_test::current_speed, robot_sdk::SpeedLevel::SPEED_LEVEL_HIGH);
    EXPECT_EQ(backend.state().mode, 3);
    expectMove(3.0);
}

TEST_F(D1MaxGaitTest, NormalGaitsLeaveEachSpecialPostureViaStandUp) {
    using robot_sdk::MotionStatus;
    for (auto source : {MotionStatus::MOTION_STATUS_GAIT, MotionStatus::MOTION_STATUS_STAIR,
                        MotionStatus::MOTION_STATUS_CRAWL_WALK, MotionStatus::MOTION_STATUS_CLIMB,
                        MotionStatus::MOTION_STATUS_SLIM, MotionStatus::MOTION_STATUS_DSB,
                        MotionStatus::MOTION_STATUS_BALANCE_STAND}) {
        SCOPED_TRACE(static_cast<int>(source));
        for (int gait : {0, 3, 34}) {
            SCOPED_TRACE(gait);
            sdk_gait_test::posture(source);
            sdk_gait_test::commands.clear();
            ASSERT_TRUE(backend.setGait(gait).success);
            EXPECT_EQ(sdk_gait_test::commands,
                      (std::vector<std::string>{"Move", "StandUp", "SetSpeed"}));
            EXPECT_EQ(sdk_gait_test::current_posture, MotionStatus::MOTION_STATUS_WALK);
            EXPECT_EQ(backend.state().mode, gait);
            const int speed = gait == 0 ? 1 : (gait == 3 ? 3 : 2);
            EXPECT_EQ(sdk_gait_test::speeds.back(), speed);
            expectMove(speed);
        }
    }
}

TEST_F(D1MaxGaitTest, StandSendErrorDoesNotSetSpeedOrReportRun) {
    ASSERT_TRUE(backend.setGait(33).success);
    sdk_gait_test::speeds.clear();
    sdk_gait_test::stand_error = std::make_error_code(std::errc::io_error);
    EXPECT_FALSE(backend.setGait(3).success);
    EXPECT_TRUE(sdk_gait_test::speeds.empty());
    EXPECT_EQ(backend.state().mode, 33);
}

TEST_F(D1MaxGaitTest, StandFeedbackMustBeWalkBeforeSettingSpeed) {
    for (auto feedback : {robot_sdk::MotionStatus::MOTION_STATUS_GAIT,
                          robot_sdk::MotionStatus::MOTION_STATUS_STAND_UP,
                          robot_sdk::MotionStatus::MOTION_STATUS_BALANCE_STAND}) {
        SCOPED_TRACE(static_cast<int>(feedback));
        sdk_gait_test::posture(robot_sdk::MotionStatus::MOTION_STATUS_GAIT);
        sdk_gait_test::stand_feedback = feedback;
        const auto result = backend.setGait(3);
        EXPECT_FALSE(result.success);
        EXPECT_NE(result.message.find("normal WALK posture after StandUp"), std::string::npos);
        EXPECT_TRUE(sdk_gait_test::speeds.empty());
        EXPECT_EQ(backend.state().mode, 33);
    }
}

TEST_F(D1MaxGaitTest, CachedHighSpeedCannotConfirmNewRunRequestWithoutFreshFeedback) {
    ASSERT_TRUE(backend.setGait(33).success);
    sdk_gait_test::acknowledge_speed = false;
    const auto result = backend.setGait(3);
    EXPECT_FALSE(result.success);
    EXPECT_NE(result.message.find("speed level feedback"), std::string::npos);
    EXPECT_EQ(backend.state().mode, 33);
}

TEST_F(D1MaxGaitTest, WrongSpeedFeedbackPreservesPreviousGaitAndNormalization) {
    ASSERT_TRUE(backend.setGait(0).success);
    sdk_gait_test::speed_feedback_override = 1;
    const auto result = backend.setGait(3);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(backend.state().mode, 0);
    expectMove(1.0);
}

TEST_F(D1MaxGaitTest, NormalSpeedSendFailureDoesNotCommitTargetGait) {
    ASSERT_TRUE(backend.setGait(0).success);
    sdk_gait_test::speed_error = std::make_error_code(std::errc::io_error);
    EXPECT_FALSE(backend.setGait(3).success);
    EXPECT_EQ(backend.state().mode, 0);
    expectMove(1.0);
}

struct YawScalingCase {
    int speed;
    double forward;
    float sdk_forward;
    float sdk_yaw;  // Expected percentage for a requested 0.3 rad/s.
};

class D1MaxYawScalingTest : public D1MaxGaitTest,
                          public testing::WithParamInterface<YawScalingCase> {};

TEST_P(D1MaxYawScalingTest, ScalesYawForForwardAndReverseWithoutDisablingLateral) {
    const auto sample = GetParam();
    ASSERT_TRUE(backend.setSpeed(sample.speed).success);
    for (double direction : {-1.0, 1.0}) {
        for (double turn : {-1.0, 1.0}) {
            ASSERT_TRUE(backend.move(direction * sample.forward, -0.1, turn * 0.3).success);
            const auto &move = sdk_gait_test::moves.back();
            EXPECT_FLOAT_EQ(move[0], -0.2F);
            EXPECT_FLOAT_EQ(move[1], static_cast<float>(direction) * sample.sdk_forward);
            EXPECT_FLOAT_EQ(move[2], static_cast<float>(turn) * sample.sdk_yaw);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(SpeedBands, D1MaxYawScalingTest, testing::Values(
    YawScalingCase{1, 0.0, 0.0F, 0.2F},
    YawScalingCase{1, 1.0, 1.0F, 0.2F},
    YawScalingCase{1, 4.0, 1.0F, 0.2F},
    YawScalingCase{2, 0.999, 0.4995F, 0.2F},
    YawScalingCase{2, 1.0, 0.5F, 0.3F},
    YawScalingCase{2, 1.001, 0.5005F, 0.3F},
    YawScalingCase{2, 2.0, 1.0F, 0.3F},
    YawScalingCase{2, 4.0, 1.0F, 0.3F},
    YawScalingCase{3, 0.999, 0.333F, 0.2F},
    YawScalingCase{3, 1.0, 1.0F / 3.0F, 0.3F},
    YawScalingCase{3, 1.5, 0.5F, 0.3F},
    YawScalingCase{3, 1.999, 1.999F / 3.0F, 0.3F},
    YawScalingCase{3, 2.0, 2.0F / 3.0F, 0.6F},
    YawScalingCase{3, 2.001, 0.667F, 0.6F},
    YawScalingCase{3, 2.5, 2.5F / 3.0F, 0.6F},
    YawScalingCase{3, 4.0, 1.0F, 0.6F}));

TEST_F(D1MaxGaitTest, YawRequestsSaturateAtCurrentBandLimit) {
    ASSERT_TRUE(backend.setSpeed(3).success);
    ASSERT_TRUE(backend.move(2.5, 0.0, 0.6).success);
    EXPECT_FLOAT_EQ(sdk_gait_test::moves.back()[2], 1.0F);
    ASSERT_TRUE(backend.move(-2.5, 0.0, -0.6).success);
    EXPECT_FLOAT_EQ(sdk_gait_test::moves.back()[2], -1.0F);
}

TEST_F(D1MaxGaitTest, GaitSelectionsUseMediumOrSlowYawScaling) {
    ASSERT_TRUE(backend.setSpeed(3).success);
    ASSERT_TRUE(backend.setGait(32).success);
    ASSERT_TRUE(backend.move(2.5, 0.0, 0.3).success);
    EXPECT_FLOAT_EQ(sdk_gait_test::moves.back()[1], 1.0F);
    EXPECT_FLOAT_EQ(sdk_gait_test::moves.back()[2], 0.3F);
    for (int gait : {6, 7, 8, 36, d1_max_gait::CRAWL_WALK, d1_max_gait::CLIMB,
                     d1_max_gait::SLIM, d1_max_gait::DSB}) {
        // Stair/special normalization must ignore a previously selected high gear.
        ASSERT_TRUE(backend.setSpeed(3).success);
        sdk_gait_test::speeds.clear();
        ASSERT_TRUE(backend.setGait(gait).success);
        EXPECT_TRUE(sdk_gait_test::speeds.empty());
        ASSERT_TRUE(backend.move(2.5, 0.0, 0.3).success);
        EXPECT_FLOAT_EQ(sdk_gait_test::moves.back()[1], 1.0F);
        EXPECT_FLOAT_EQ(sdk_gait_test::moves.back()[2], 0.2F);
    }
}
}  // namespace nav_bridge
