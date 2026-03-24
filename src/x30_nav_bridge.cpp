/// @file x30_nav_bridge.cpp
/// @brief 绝影X30导航桥接实现 — UDP模拟手柄轴指令, 接收IMU/里程计/状态数据

#include "nav_bridge/x30_nav_bridge.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace nav_bridge {

using namespace x30_protocol;

// ============================================================================
// 构造 / 析构
// ============================================================================

X30NavBridge::X30NavBridge(const rclcpp::NodeOptions &options)
    : NavBridgeBase("nav_bridge_node", options) {
    // 声明参数
    this->declare_parameter("motion_host_ip", std::string(MOTION_HOST_IP));
    this->declare_parameter("motion_host_port", MOTION_HOST_PORT);
    this->declare_parameter("local_recv_port", 43897);
    this->declare_parameter("heartbeat_interval_ms", HEARTBEAT_INTERVAL_MS);
    this->declare_parameter("cmd_vel_rate_hz", CMD_VEL_RATE_HZ);
    this->declare_parameter("cmd_vel_timeout_ms", 500);
    this->declare_parameter("imu_frame_id", std::string("imu_link"));
    this->declare_parameter("odom_frame_id", std::string("odom"));
    this->declare_parameter("base_frame_id", std::string("base_link"));
    this->declare_parameter("publish_tf", true);

    // 读取参数
    motion_host_ip_        = this->get_parameter("motion_host_ip").as_string();
    motion_host_port_      = this->get_parameter("motion_host_port").as_int();
    local_recv_port_       = this->get_parameter("local_recv_port").as_int();
    heartbeat_interval_ms_ = this->get_parameter("heartbeat_interval_ms").as_int();
    cmd_vel_rate_hz_       = this->get_parameter("cmd_vel_rate_hz").as_int();
    cmd_vel_timeout_ms_    = this->get_parameter("cmd_vel_timeout_ms").as_int();
    imu_frame_id_          = this->get_parameter("imu_frame_id").as_string();
    odom_frame_id_         = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_         = this->get_parameter("base_frame_id").as_string();
    publish_tf_            = this->get_parameter("publish_tf").as_bool();

    control_session_.configure(heartbeat_interval_ms_, cmd_vel_timeout_ms_);
}

X30NavBridge::~X30NavBridge() {
    shutdown();
}

// ============================================================================
// 生命周期
// ============================================================================

bool X30NavBridge::initialize() {
    RCLCPP_INFO(this->get_logger(), "=== 绝影X30 Nav Bridge 初始化 ===");
    RCLCPP_INFO(this->get_logger(), "运动主机: %s:%d", motion_host_ip_.c_str(), motion_host_port_);
    RCLCPP_INFO(this->get_logger(), "本地接收端口: %d", local_recv_port_);

    // 打印结构体大小，方便调试协议对齐
    RCLCPP_INFO(this->get_logger(),
                "协议结构体大小: CommandHead=%zu, RcsData=%zu, MotionStateData=%zu, "
                "ControllerSensorData=%zu, BatterySensorData=%zu",
                sizeof(CommandHead), sizeof(RcsData), sizeof(MotionStateData),
                sizeof(ControllerSensorData), sizeof(BatterySensorData));

    // 1. 打开UDP — 连接运动主机, 同时绑定本地端口用于接收
    if (!motion_udp_.open(motion_host_ip_, motion_host_port_, local_recv_port_)) {
        RCLCPP_ERROR(this->get_logger(), "无法打开UDP socket (运动主机)");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "UDP socket 已打开");

    // 2. 创建ROS2话题
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->cmdVelCallback(msg); });

    imu_pub_           = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    odom_pub_          = this->create_publisher<nav_msgs::msg::Odometry>("/leg_odom", 10);
    robot_state_pub_   = this->create_publisher<std_msgs::msg::Int32>("/robot_basic_state", 10);
    gait_state_pub_    = this->create_publisher<std_msgs::msg::Int32>("/robot_gait_state", 10);
    battery_level_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/battery/level", 10);

    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    using namespace std::placeholders;
    stand_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stand", std::bind(&X30NavBridge::handleStandRequest, this, _1, _2));
    lie_down_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/lie_down", std::bind(&X30NavBridge::handleLieDownRequest, this, _1, _2));
    force_stand_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/force_stand", std::bind(&X30NavBridge::handleForceStandRequest, this, _1, _2));
    ready_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/ready", std::bind(&X30NavBridge::handleReadyRequest, this, _1, _2));

    action_executor_ = std::make_unique<ActionExecutor>(
        this->get_logger(), state_store_, control_session_,
        [this](const ControlActions &actions) { this->applyControlActions(actions); },
        [this](uint32_t code) { this->sendGaitCommand(code); });

    // 3. 启动统一控制定时器 (速度发送 + 心跳 + 超时检测)
    running_ = true;
    int cmd_period_ms = 1000 / cmd_vel_rate_hz_;
    cmd_vel_timer_    = this->create_wall_timer(std::chrono::milliseconds(cmd_period_ms),
                                                [this]() { this->sendCmdVelTick(); });

    // 4. 启动接收线程
    recv_thread_ = std::thread(&X30NavBridge::receiveLoop, this);

    RCLCPP_INFO(this->get_logger(), "=== Nav Bridge 初始化完成, 等待机器狗连接... ===");
    return true;
}

void X30NavBridge::shutdown() {
    if (!running_.load()) return;  // 防止重复关闭

    RCLCPP_INFO(this->get_logger(), "Nav Bridge 关闭中...");

    running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }

    // 发送速度归零
    if (motion_udp_.isOpen()) {
        auto zero_cmd = makeSimpleCommand(CMD_VEL_FORWARD, 0);
        motion_udp_.send(&zero_cmd, sizeof(zero_cmd));
        zero_cmd = makeSimpleCommand(CMD_VEL_LATERAL, 0);
        motion_udp_.send(&zero_cmd, sizeof(zero_cmd));
        zero_cmd = makeSimpleCommand(CMD_VEL_YAW, 0);
        motion_udp_.send(&zero_cmd, sizeof(zero_cmd));
    }

    control_session_.stop();

    motion_udp_.close();
    percept_udp_.close();

    RCLCPP_INFO(this->get_logger(), "Nav Bridge 已关闭");
}

// ============================================================================
// 下行: 速度指令
// ============================================================================

void X30NavBridge::sendVelocityCommand(double vx, double vy, double vyaw) {
    // 根据当前步态获取速度上限
    auto limits = getGaitSpeedLimit(state_store_.gaitState());

    // 映射到轴指令值
    // 注意: vx 对应 CMD_VEL_FORWARD (左摇杆Y), 正值前移
    //       vy 对应 CMD_VEL_LATERAL (左摇杆X), 狗的控制中正值右移负值左移, 而ROS
    //       Twist中正数意味着左平移, 因此需取反 vyaw 对应 CMD_VEL_YAW (右摇杆X),
    //       狗的控制中正值右转负数左转, 而ROS Twist中正数意味着左侧旋转, 因此需取反
    float max_vx     = (vx >= 0) ? limits.max_forward : limits.max_backward;
    int32_t raw_vx   = velocityToAxisValue(static_cast<float>(vx), max_vx);
    int32_t raw_vy   = velocityToAxisValue(static_cast<float>(-vy), limits.max_lateral);
    int32_t raw_vyaw = velocityToAxisValue(static_cast<float>(-vyaw), limits.max_yaw);

    // 发送三条轴指令
    auto cmd_fwd = makeSimpleCommand(CMD_VEL_FORWARD, raw_vx);
    auto cmd_lat = makeSimpleCommand(CMD_VEL_LATERAL, raw_vy);
    auto cmd_yaw = makeSimpleCommand(CMD_VEL_YAW, raw_vyaw);

    motion_udp_.send(&cmd_fwd, sizeof(cmd_fwd));
    motion_udp_.send(&cmd_lat, sizeof(cmd_lat));
    motion_udp_.send(&cmd_yaw, sizeof(cmd_yaw));
}

void X30NavBridge::sendGaitCommand(uint32_t gait_cmd_code) {
    auto cmd = makeSimpleCommand(gait_cmd_code, 0);
    motion_udp_.send(&cmd, sizeof(cmd));
    RCLCPP_INFO(this->get_logger(), "发送步态指令: 0x%08X", gait_cmd_code);
}

// ============================================================================
// 定频发送轴指令
// ============================================================================

void X30NavBridge::applyControlActions(const ControlActions &actions) {
    if (actions.send_heartbeat) {
        auto hb = makeSimpleCommand(CMD_HEARTBEAT, 0);
        motion_udp_.send(&hb, sizeof(hb));
    }

    if (actions.send_query) {
        auto query = makeSimpleCommand(CMD_QUERY_103, 0);
        motion_udp_.send(&query, sizeof(query));
        RCLCPP_INFO(this->get_logger(), "心跳已启动, 并已发送连接确认查询(0x21020001)");
    }

    if (actions.send_zero_velocity) {
        sendVelocityCommand(0.0, 0.0, 0.0);
    }
}

void X30NavBridge::sendCmdVelTick() {
    auto now       = std::chrono::steady_clock::now();
    auto last_recv = state_store_.lastReceiveTime();
    if (state_store_.connected()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_recv).count();
        if (elapsed > 2000) {
            state_store_.markDisconnected();
            robot_state_.store(RobotState::DISCONNECTED);
            RCLCPP_WARN(this->get_logger(), "⚠️ 机器狗连接丢失! (%.1f秒无数据)", elapsed / 1000.0);
        }
    }

    ControlActions actions = control_session_.tick(now);
    applyControlActions(actions);
    if (actions.session_stopped) {
        RCLCPP_INFO(this->get_logger(), "🎮 控制超时, 停止心跳, 遥控器恢复控制");
        return;
    }

    if (!actions.active) {
        return;
    }

    if (actions.session_started) {
        RCLCPP_INFO(this->get_logger(), "🤖 控制会话激活, 执行控制权获取...");
        RCLCPP_INFO(this->get_logger(), "🤖 控制权获取完成, 开始自主控制");
    }

    double vx   = target_vx_.load();
    double vy   = target_vy_.load();
    double vyaw = target_vyaw_.load();
    sendVelocityCommand(vx, vy, vyaw);
}

// ============================================================================
// 上行: 接收线程
// ============================================================================

void X30NavBridge::receiveLoop() {
    RCLCPP_INFO(this->get_logger(), "接收线程启动");

    CommandMessage msg;
    uint64_t total_recv_count   = 0;
    uint64_t unknown_code_count = 0;

    while (running_) {
        int received = motion_udp_.receive(&msg, sizeof(msg), 50);  // 50ms超时
        if (received <= 0) {
            continue;
        }

        // 至少要有CommandHead大小
        if (received < static_cast<int>(sizeof(CommandHead))) {
            continue;
        }

        total_recv_count++;

        // 更新最后接收时间 & 连接状态
        bool was_connected = state_store_.connected();
        state_store_.markPacketReceived(std::chrono::steady_clock::now());
        if (!was_connected) {
            RCLCPP_INFO(this->get_logger(), "✅ 已连接到机器狗 (首包 code=0x%04X, 大小=%d bytes)",
                        msg.head.code, received);
        }

        // 获取数据载荷大小 (总接收 - CommandHead)
        int data_size = received - static_cast<int>(sizeof(CommandHead));

        // 根据指令码分发
        switch (msg.head.code) {
            case RECV_RCS_DATA: {
                if (msg.head.type == 1 && data_size >= static_cast<int>(sizeof(RcsData))) {
                    const auto *data = reinterpret_cast<const RcsData *>(msg.data_buffer);
                    handleRcsData(*data);
                } else {
                    RCLCPP_WARN_ONCE(this->get_logger(),
                                     "RcsData 大小不匹配: 收到 data=%d bytes, 期望 >=%zu bytes",
                                     data_size, sizeof(RcsData));
                }
                break;
            }
            case RECV_MOTION_STATE: {
                if (msg.head.type == 1 && data_size >= static_cast<int>(sizeof(MotionStateData))) {
                    const auto *data = reinterpret_cast<const MotionStateData *>(msg.data_buffer);
                    handleMotionState(*data);
                } else {
                    RCLCPP_WARN_ONCE(
                        this->get_logger(),
                        "MotionStateData 大小不匹配: 收到 data=%d bytes, 期望 >=%zu bytes",
                        data_size, sizeof(MotionStateData));
                }
                break;
            }
            case RECV_CONTROLLER_SENSOR: {
                if (msg.head.type == 1 &&
                    data_size >= static_cast<int>(sizeof(ControllerSensorData))) {
                    const auto *data =
                        reinterpret_cast<const ControllerSensorData *>(msg.data_buffer);
                    handleControllerSensor(*data);
                } else {
                    RCLCPP_WARN_ONCE(
                        this->get_logger(),
                        "ControllerSensorData 大小不匹配: 收到 data=%d bytes, 期望 >=%zu bytes",
                        data_size, sizeof(ControllerSensorData));
                }
                break;
            }
            case RECV_BATTERY: {
                if (msg.head.type == 1 &&
                    data_size >= static_cast<int>(sizeof(BatterySensorData))) {
                    const auto *data = reinterpret_cast<const BatterySensorData *>(msg.data_buffer);
                    handleBattery(*data);
                } else {
                    RCLCPP_WARN_ONCE(
                        this->get_logger(),
                        "BatterySensorData 大小不匹配: 收到 data=%d bytes, 期望 >=%zu bytes",
                        data_size, sizeof(BatterySensorData));
                }
                break;
            }
            default: {
                unknown_code_count++;
                if (unknown_code_count <= 5) {
                    RCLCPP_DEBUG(this->get_logger(),
                                 "未处理的指令码: 0x%04X, type=%u, data_size=%d", msg.head.code,
                                 msg.head.type, data_size);
                }
                break;
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "接收线程退出 (共接收 %lu 包, %lu 未知指令码)",
                total_recv_count, unknown_code_count);
}

void X30NavBridge::processIncomingData() {
    // 接收逻辑在独立线程中运行, 此方法保留接口兼容
}

// ============================================================================
// 数据处理: RcsData (运行状态, 0x1008)
// ============================================================================

void X30NavBridge::handleRcsData(const RcsData &data) {
    // 首次收到 RcsData 时打印机器人名称
    if (state_store_.markRcsReceived()) {
        RCLCPP_INFO(this->get_logger(), "🐕 机器人名称: %.*s", 15, data.robot_name);
        RCLCPP_INFO(this->get_logger(), "   控制模式: %s, 累计里程: %.1f m, 累计运行: %ld s",
                    data.rcs_state_list.is_nav_mode ? "非手动" : "手动", data.total_mileage / 100.0,
                    data.total_run_time);
    }

    // 检查错误状态
    if (data.error_state != 0) {
        if (data.error_state_bit.battery_low_warn) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "⚠️ 电池低电量警告!");
        }
        if (data.error_state_bit.imu_error) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "❌ IMU 更新超时!");
        }
        if (data.error_state_bit.driver_error) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "❌ 驱动器故障!");
        }
        if (data.error_state_bit.motor_heat_warn) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "⚠️ 电机过温警告!");
        }
    }
}

// ============================================================================
// 数据处理: MotionStateData (运动状态, 0x1009)
// ============================================================================

/// 基本状态枚举转文字
static const char *basicStateToStr(uint8_t state) {
    switch (static_cast<BasicState>(state)) {
        case BasicState::LYING_DOWN:
            return "趴下";
        case BasicState::STANDING_UP:
            return "正在起立";
        case BasicState::INITIAL_STAND:
            return "初始站立";
        case BasicState::FORCE_STAND:
            return "力控站立";
        case BasicState::STEPPING:
            return "踏步运动";
        case BasicState::GOING_DOWN:
            return "正在趴下";
        case BasicState::SOFT_ESTOP:
            return "软急停/摔倒";
        case BasicState::RL_MODE:
            return "RL模式";
        default:
            return "未知";
    }
}

/// 步态枚举转文字
static const char *gaitStateToStr(uint8_t state) {
    switch (static_cast<GaitState>(state)) {
        case GaitState::WALK:
            return "行走";
        case GaitState::OBSTACLE:
            return "越障";
        case GaitState::SLOPE:
            return "斜坡";
        case GaitState::RUN:
            return "跑步";
        case GaitState::STAIR_SOLID:
            return "楼梯";
        case GaitState::STAIR_ACC:
            return "楼梯(累积帧)";
        case GaitState::STAIR45_ACC:
            return "45°楼梯(累积帧)";
        case GaitState::L_WALK:
            return "L行走";
        case GaitState::MOUNTAIN:
            return "山地";
        case GaitState::SILENT:
            return "静音";
        default:
            return "未知";
    }
}

void X30NavBridge::handleMotionState(const MotionStateData &data) {
    rclcpp::Time now = this->now();

    MotionStateTransition transition = state_store_.updateMotionState(data.basic_state, data.gait_state);

    // 状态变化时打印日志
    if (data.basic_state != transition.previous_basic_state) {
        RCLCPP_INFO(this->get_logger(), "🔄 基本状态: %s → %s", basicStateToStr(transition.previous_basic_state),
                    basicStateToStr(data.basic_state));
    }
    if (data.gait_state != transition.previous_gait_state) {
        RCLCPP_INFO(this->get_logger(), "🔄 步态: %s → %s", gaitStateToStr(transition.previous_gait_state),
                    gaitStateToStr(data.gait_state));
    }

    // 发布基本状态
    auto state_msg = std_msgs::msg::Int32();
    state_msg.data = data.basic_state;
    robot_state_pub_->publish(state_msg);

    // 发布步态状态
    auto gait_msg = std_msgs::msg::Int32();
    gait_msg.data = data.gait_state;
    gait_state_pub_->publish(gait_msg);

    // 更新robot_state_枚举
    switch (static_cast<BasicState>(data.basic_state)) {
        case BasicState::LYING_DOWN:
            robot_state_.store(RobotState::LYING_DOWN);
            break;
        case BasicState::STANDING_UP:
            robot_state_.store(RobotState::STANDING_UP);
            break;
        case BasicState::INITIAL_STAND:
            robot_state_.store(RobotState::INITIAL_STAND);
            break;
        case BasicState::FORCE_STAND:
            robot_state_.store(RobotState::FORCE_STAND);
            break;
        case BasicState::STEPPING:
            robot_state_.store(RobotState::STEPPING);
            break;
        case BasicState::GOING_DOWN:
            robot_state_.store(RobotState::GOING_DOWN);
            break;
        case BasicState::SOFT_ESTOP:
            robot_state_.store(RobotState::SOFT_ESTOP);
            break;
        case BasicState::RL_MODE:
            robot_state_.store(RobotState::RL_MODE);
            break;
        default:
            break;
    }

    // 发布 Odometry
    auto odom_msg            = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = now;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id  = base_frame_id_;

    // 位置 (只有 x, y, yaw — 2D 里程计)
    odom_msg.pose.pose.position.x = data.leg_odom_pos[0];
    odom_msg.pose.pose.position.y = data.leg_odom_pos[1];
    odom_msg.pose.pose.position.z = 0.0;

    // yaw → quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, data.leg_odom_pos[2]);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 速度 (body frame)
    odom_msg.twist.twist.linear.x  = data.leg_odom_vel[0];
    odom_msg.twist.twist.linear.y  = data.leg_odom_vel[1];
    odom_msg.twist.twist.angular.z = data.leg_odom_vel[2];

    odom_pub_->publish(odom_msg);

    // 发布 TF: odom → base_link
    if (publish_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp            = now;
        tf.header.frame_id         = odom_frame_id_;
        tf.child_frame_id          = base_frame_id_;
        tf.transform.translation.x = data.leg_odom_pos[0];
        tf.transform.translation.y = data.leg_odom_pos[1];
        tf.transform.translation.z = 0.0;
        tf.transform.rotation      = odom_msg.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    // 周期性诊断日志 (每10秒)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "📊 状态=%s 步态=%s | 位置=(%.2f, %.2f) yaw=%.1f° | 速度=(%.2f, %.2f) "
                         "vyaw=%.2f | 里程=%.1fm",
                         basicStateToStr(data.basic_state), gaitStateToStr(data.gait_state),
                         data.leg_odom_pos[0], data.leg_odom_pos[1],
                         data.leg_odom_pos[2] * 180.0 / M_PI, data.leg_odom_vel[0],
                         data.leg_odom_vel[1], data.leg_odom_vel[2], data.robot_distance / 100.0);
}

// ============================================================================
// 数据处理: ControllerSensorData (传感器, 0x100A)
// ============================================================================

void X30NavBridge::handleControllerSensor(const ControllerSensorData &data) {
    rclcpp::Time now = this->now();

    const auto &imu = data.imu_data;

    auto imu_msg            = sensor_msgs::msg::Imu();
    imu_msg.header.stamp    = now;
    imu_msg.header.frame_id = imu_frame_id_;

    // 欧拉角 → quaternion (规格书: roll/pitch/yaw 单位是度)
    constexpr double DEG2RAD = M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(imu.roll * DEG2RAD, imu.pitch * DEG2RAD, imu.yaw * DEG2RAD);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    // 角速度 (rad/s — 规格书原始单位就是 rad/s)
    imu_msg.angular_velocity.x = imu.omega_x;
    imu_msg.angular_velocity.y = imu.omega_y;
    imu_msg.angular_velocity.z = imu.omega_z;

    // 线加速度 (m/s² — 规格书原始单位就是 m/s²)
    imu_msg.linear_acceleration.x = imu.acc_x;
    imu_msg.linear_acceleration.y = imu.acc_y;
    imu_msg.linear_acceleration.z = imu.acc_z;

    imu_pub_->publish(imu_msg);

    // 周期性 IMU 诊断 (每 10 秒)
    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "IMU: RPY=(%.1f°, %.1f°, %.1f°) ω=(%.3f, %.3f, %.3f) a=(%.2f, %.2f, %.2f)", imu.roll,
        imu.pitch, imu.yaw, imu.omega_x, imu.omega_y, imu.omega_z, imu.acc_x, imu.acc_y, imu.acc_z);
}

// ============================================================================
// 数据处理: BatterySensorData (电池, 0x21050F0A)
// ============================================================================

void X30NavBridge::handleBattery(const BatterySensorData &data) {
    auto msg = std_msgs::msg::UInt8();
    msg.data = data.battery_level;
    battery_level_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000, "🔋 电池: %d%%, 电压: %dV",
                         data.battery_level, data.voltage);
}

// ============================================================================
// 服务: 动作控制 (阻塞等待状态反馈)
// ============================================================================

void X30NavBridge::handleStandRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    auto result  = action_executor_->stand();
    res->success = result.success;
    res->message = result.message;
}

void X30NavBridge::handleLieDownRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    auto result  = action_executor_->lieDown();
    res->success = result.success;
    res->message = result.message;
}

void X30NavBridge::handleForceStandRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    auto result  = action_executor_->forceStand();
    res->success = result.success;
    res->message = result.message;
}

void X30NavBridge::handleReadyRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    auto result  = action_executor_->ready();
    res->success = result.success;
    res->message = result.message;
}

// ============================================================================
// 基类回调
// ============================================================================

void X30NavBridge::onControlInputUpdated() {
    control_session_.noteActivity(std::chrono::steady_clock::now());
}

void NavBridgeBase::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_vx_.store(msg->linear.x);
    target_vy_.store(msg->linear.y);
    target_vyaw_.store(msg->angular.z);

    // 更新最后一次收到有效指令的时间
    last_active_time_.store(std::chrono::steady_clock::now());
    onControlInputUpdated();
}

NavBridgeBase::NavBridgeBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options) {}

}  // namespace nav_bridge
