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
    motion_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/motion", std::bind(&X30NavBridge::handleMotionRequest, this, _1, _2));

    // 3. 启动心跳定时器
    heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(heartbeat_interval_ms_),
                                               [this]() { this->sendHeartbeat(); });

    // 4. 启动轴指令定频发送定时器
    int cmd_period_ms = 1000 / cmd_vel_rate_hz_;
    cmd_vel_timer_    = this->create_wall_timer(std::chrono::milliseconds(cmd_period_ms),
                                                [this]() { this->sendCmdVelTick(); });

    // 5. 启动接收线程
    running_     = true;
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

    motion_udp_.close();
    percept_udp_.close();

    RCLCPP_INFO(this->get_logger(), "Nav Bridge 已关闭");
}

// ============================================================================
// 下行: 速度指令
// ============================================================================

void X30NavBridge::sendVelocityCommand(double vx, double vy, double vyaw) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 根据当前步态获取速度上限
    auto limits = getGaitSpeedLimit(current_gait_state_);

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
// 下行: 心跳
// ============================================================================

void X30NavBridge::sendHeartbeat() {
    bool is_active = control_active_.load();

    // 1. 如果没有任何控制输入（速度或服务指令），则停止发送心跳，将控制权还给物理遥控器
    if (!is_active) {
        // 首次心跳标志位复位，以便下一次接管时重新确认
        heartbeat_confirmed_ = false;
    } else {
        // 有控制输入时，维持心跳，接管控制权
        auto cmd = makeSimpleCommand(CMD_HEARTBEAT, 0);
        motion_udp_.send(&cmd, sizeof(cmd));

        // 首次心跳后发送查询指令确认连接
        if (!heartbeat_confirmed_) {
            auto query = makeSimpleCommand(CMD_QUERY_103, 0);
            motion_udp_.send(&query, sizeof(query));
            heartbeat_confirmed_ = true;
            RCLCPP_INFO(this->get_logger(), "心跳已启动, 已发送连接确认查询");
        }
    }

    // ===== 断连检测 =====
    auto now           = std::chrono::steady_clock::now();
    auto last          = last_recv_time_.load();
    bool was_connected = connected_.load();

    if (was_connected) {
        // 超过 2 秒没有收到任何数据 → 判定断连
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
        if (elapsed > 2000) {
            connected_.store(false);
            robot_state_.store(RobotState::DISCONNECTED);
            RCLCPP_WARN(this->get_logger(), "⚠️ 机器狗连接丢失! (%.1f秒无数据)", elapsed / 1000.0);
        }
    }
}

// ============================================================================
// 定频发送轴指令
// ============================================================================

void X30NavBridge::sendCmdVelTick() {
    // 检查控制是否活跃 (通过 last_active_time_ 统一判断)
    auto now        = std::chrono::steady_clock::now();
    auto last_act   = last_active_time_.load();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_act).count();

    bool was_active = control_active_.load();
    bool is_active  = false;

    if (last_act != std::chrono::steady_clock::time_point::min()) {
        is_active = (elapsed_ms >= 0 && elapsed_ms < cmd_vel_timeout_ms_);
    }

    if (was_active && !is_active) {
        // 从活跃→非活跃: 发送一次归零, 然后停止发送, 交还遥控器控制
        sendVelocityCommand(0.0, 0.0, 0.0);
        control_active_.store(false);
        RCLCPP_INFO(this->get_logger(), "🎮 控制超时, 停止发送指令, 遥控器恢复控制");
        return;
    }

    if (!is_active) {
        // 非活跃状态: 不发送任何控制指令, 让遥控器正常工作
        return;
    }

    if (!was_active && is_active) {
        control_active_.store(true);
        RCLCPP_INFO(this->get_logger(), "🤖 控制接入, 开始自主控制");
    }

    // 活跃状态: 持续发送速度指令 (包括零速也要持续发, 否则机器狗超时停止)
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
        last_recv_time_.store(std::chrono::steady_clock::now());
        if (!connected_.load()) {
            connected_.store(true);
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
    if (!rcs_received_) {
        rcs_received_ = true;
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

    // 更新内部状态
    uint8_t prev_basic, prev_gait;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        prev_basic           = current_basic_state_;
        prev_gait            = current_gait_state_;
        current_basic_state_ = data.basic_state;
        current_gait_state_  = data.gait_state;
    }

    // 状态变化时打印日志
    if (data.basic_state != prev_basic) {
        RCLCPP_INFO(this->get_logger(), "🔄 基本状态: %s → %s", basicStateToStr(prev_basic),
                    basicStateToStr(data.basic_state));
    }
    if (data.gait_state != prev_gait) {
        RCLCPP_INFO(this->get_logger(), "🔄 步态: %s → %s", gaitStateToStr(prev_gait),
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

/// 确保在发送步态指令前已经注册了控制权
/// 冷启动时发送更多心跳以确保机器人注册成功
void X30NavBridge::ensureControlTakeover() {
    last_active_time_.store(std::chrono::steady_clock::now());

    bool was_active = control_active_.load();

    // 立即标记为活跃, 这样 sendHeartbeat 定时器也会参与发送心跳
    if (!was_active) {
        control_active_.store(true);
    }

    // 冷启动时发送更多心跳, 给机器人足够时间注册控制权
    int count = was_active ? 3 : 50;  // 冷启动2500ms, 热启动150ms

    for (int i = 0; i < count; ++i) {
        auto hb = makeSimpleCommand(CMD_HEARTBEAT, 0);
        motion_udp_.send(&hb, sizeof(hb));

        if (!heartbeat_confirmed_) {
            auto query = makeSimpleCommand(CMD_QUERY_103, 0);
            motion_udp_.send(&query, sizeof(query));
            heartbeat_confirmed_ = true;
            RCLCPP_INFO(this->get_logger(), "心跳已启动, 已发送连接确认查询");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        last_active_time_.store(std::chrono::steady_clock::now());
    }
}

/// 等待机器人基本状态到达目标状态之一
/// @param targets    目标状态集合 (任一匹配即返回成功)
/// @param timeout_ms 最大等待时间
/// @return true=到达目标状态, false=超时
bool X30NavBridge::waitForBasicState(const std::vector<BasicState> &targets, int timeout_ms) {
    constexpr int POLL_INTERVAL_MS = 50;  // 轮询间隔
    constexpr int HB_EVERY         = 4;   // 每4次轮询发一次心跳 (~200ms)
    int elapsed                    = 0;
    int poll_count                 = 0;

    while (elapsed < timeout_ms) {
        // 持续刷新活跃时间
        last_active_time_.store(std::chrono::steady_clock::now());

        // 内联发送心跳 (服务回调阻塞了执行器, 定时器不会触发)
        if (poll_count % HB_EVERY == 0) {
            auto hb = makeSimpleCommand(CMD_HEARTBEAT, 0);
            motion_udp_.send(&hb, sizeof(hb));
        }

        uint8_t state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = current_basic_state_;
        }

        for (const auto &t : targets) {
            if (state == static_cast<uint8_t>(t)) {
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
        elapsed += POLL_INTERVAL_MS;
        poll_count++;
    }
    return false;
}

void X30NavBridge::handleStandRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    uint8_t state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // 已经站着了，直接返回成功
    if (state == static_cast<uint8_t>(BasicState::INITIAL_STAND) ||
        state == static_cast<uint8_t>(BasicState::FORCE_STAND) ||
        state == static_cast<uint8_t>(BasicState::STEPPING) ||
        state == static_cast<uint8_t>(BasicState::RL_MODE)) {
        res->success = true;
        res->message = "Robot is already standing.";
        return;
    }

    // 处理软急停: 先发 CMD_STAND_UP_DOWN 转到 LYING_DOWN
    if (state == static_cast<uint8_t>(BasicState::SOFT_ESTOP)) {
        RCLCPP_INFO(this->get_logger(), "⚠️ 机器人在软急停状态, 先恢复到贴下...");
        ensureControlTakeover();
        sendGaitCommand(CMD_STAND_UP_DOWN);

        if (!waitForBasicState({BasicState::LYING_DOWN}, 5000)) {
            res->success = false;
            res->message = "Timeout recovering from soft estop to lying down.";
            RCLCPP_WARN(this->get_logger(), "⚠️ 从软急停恢复贴下超时");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 已恢复到贴下状态");
        // 更新状态继续执行下面的起立流程
        state = static_cast<uint8_t>(BasicState::LYING_DOWN);
    }

    if (state != static_cast<uint8_t>(BasicState::LYING_DOWN) &&
        state != static_cast<uint8_t>(BasicState::GOING_DOWN)) {
        res->success = false;
        res->message =
            "Robot is not in a state that can stand up (current state=" + std::to_string(state) +
            ").";
        return;
    }

    // 先注册控制权, 再发送指令
    ensureControlTakeover();
    sendGaitCommand(CMD_STAND_UP_DOWN);
    RCLCPP_INFO(this->get_logger(), "⏳ 等待机器人起立...");

    // 阻塞等待: 贴下 → 正在起立 → 初始站立
    bool ok = waitForBasicState(
        {BasicState::INITIAL_STAND, BasicState::FORCE_STAND, BasicState::STEPPING},
        8000);  // 起立可能较慢，给8秒

    if (ok) {
        res->success = true;
        res->message = "Robot has stood up.";
        RCLCPP_INFO(this->get_logger(), "✅ 起立完成");
    } else {
        res->success = false;
        res->message = "Timeout waiting for robot to stand up.";
        RCLCPP_WARN(this->get_logger(), "⚠️ 起立超时");
    }
}

void X30NavBridge::handleLieDownRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    uint8_t state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // 已经趴着了
    if (state == static_cast<uint8_t>(BasicState::LYING_DOWN)) {
        res->success = true;
        res->message = "Robot is already lying down.";
        return;
    }

    // 正在趴下, 等完成即可
    if (state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
        bool ok      = waitForBasicState({BasicState::LYING_DOWN}, 5000);
        res->success = ok;
        res->message = ok ? "Robot has lied down." : "Timeout waiting for lie down.";
        return;
    }

    // 软急停: 发 CMD_STAND_UP_DOWN 直接恢复到趴下
    if (state == static_cast<uint8_t>(BasicState::SOFT_ESTOP)) {
        ensureControlTakeover();
        sendGaitCommand(CMD_STAND_UP_DOWN);
        RCLCPP_INFO(this->get_logger(), "⏳ 从软急停恢复趴下...");
        bool ok      = waitForBasicState({BasicState::LYING_DOWN}, 5000);
        res->success = ok;
        res->message = ok ? "Robot recovered and lied down." : "Timeout recovering from estop.";
        if (ok)
            RCLCPP_INFO(this->get_logger(), "✅ 趴下完成");
        else
            RCLCPP_WARN(this->get_logger(), "⚠️ 趴下超时");
        return;
    }

    // RL模式 或 踏步运动: 先停止运动 → 力控站立, 再趴下
    if (state == static_cast<uint8_t>(BasicState::RL_MODE) ||
        state == static_cast<uint8_t>(BasicState::STEPPING)) {
        RCLCPP_INFO(this->get_logger(), "⏳ 先停止运动...");
        ensureControlTakeover();
        sendGaitCommand(CMD_STAND_UP_DOWN);

        if (!waitForBasicState({BasicState::FORCE_STAND, BasicState::INITIAL_STAND}, 5000)) {
            res->success = false;
            res->message = "Timeout stopping motion before lie down.";
            RCLCPP_WARN(this->get_logger(), "⚠️ 停止运动超时");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 已停止运动");
    }

    // 从站立状态(INITIAL_STAND/FORCE_STAND) 趴下
    ensureControlTakeover();
    sendGaitCommand(CMD_STAND_UP_DOWN);
    RCLCPP_INFO(this->get_logger(), "⏳ 等待机器人趴下...");

    bool ok = waitForBasicState({BasicState::LYING_DOWN}, 5000);

    if (ok) {
        res->success = true;
        res->message = "Robot has lied down.";
        RCLCPP_INFO(this->get_logger(), "✅ 趴下完成");
    } else {
        res->success = false;
        res->message = "Timeout waiting for robot to lie down.";
        RCLCPP_WARN(this->get_logger(), "⚠️ 趴下超时");
    }
}

void X30NavBridge::handleForceStandRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    uint8_t state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // 已经在力控站立了，直接返回成功
    if (state == static_cast<uint8_t>(BasicState::FORCE_STAND)) {
        res->success = true;
        res->message = "Robot is already in force stand mode.";
        return;
    }

    // 先注册控制权, 再发送指令
    ensureControlTakeover();
    sendGaitCommand(CMD_FORCE_CONTROL);
    RCLCPP_INFO(this->get_logger(), "⏳ 等待机器人进入力控站立...");

    // 阻塞等待: 目标状态为 FORCE_STAND
    bool ok = waitForBasicState({BasicState::FORCE_STAND}, 5000);

    if (ok) {
        res->success = true;
        res->message = "Robot is now in force stand mode.";
        RCLCPP_INFO(this->get_logger(), "✅ 力控站立完成");
    } else {
        res->success = false;
        res->message = "Timeout waiting for force stand mode.";
        RCLCPP_WARN(this->get_logger(), "⚠️ 力控站立超时");
    }
}

bool X30NavBridge::waitForGaitState(const std::vector<GaitState> &targets, int timeout_ms) {
    constexpr int POLL_INTERVAL_MS = 50;
    constexpr int HB_EVERY         = 4;  // 每4次轮询发一次心跳 (~200ms)
    int elapsed                    = 0;
    int poll_count                 = 0;

    while (elapsed < timeout_ms) {
        last_active_time_.store(std::chrono::steady_clock::now());

        // 内联发送心跳 (服务回调阻塞了执行器, 定时器不会触发)
        if (poll_count % HB_EVERY == 0) {
            auto hb = makeSimpleCommand(CMD_HEARTBEAT, 0);
            motion_udp_.send(&hb, sizeof(hb));
        }

        uint8_t gait;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            gait = current_gait_state_;
        }

        for (const auto &t : targets) {
            if (gait == static_cast<uint8_t>(t)) {
                return true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
        elapsed += POLL_INTERVAL_MS;
        poll_count++;
    }
    return false;
}

void X30NavBridge::handleMotionRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    uint8_t state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // 已经在踏步状态了
    if (state == static_cast<uint8_t>(BasicState::STEPPING)) {
        res->success = true;
        res->message = "Robot is already in motion (stepping).";
        return;
    }

    // 需要在力控站立、初始站立或RL状态才能开始运动
    if (state != static_cast<uint8_t>(BasicState::FORCE_STAND) &&
        state != static_cast<uint8_t>(BasicState::INITIAL_STAND) &&
        state != static_cast<uint8_t>(BasicState::RL_MODE)) {
        res->success = false;
        res->message =
            "Robot must be standing to start motion (current state=" + std::to_string(state) + ").";
        return;
    }

    ensureControlTakeover();
    sendGaitCommand(CMD_MOTION);
    RCLCPP_INFO(this->get_logger(), "⏳ 等待机器人开始运动...");

    bool ok = waitForBasicState({BasicState::STEPPING}, 5000);

    if (ok) {
        res->success = true;
        res->message = "Robot is now in motion.";
        RCLCPP_INFO(this->get_logger(), "✅ 运动已启动");
    } else {
        res->success = false;
        res->message = "Timeout waiting for motion to start.";
        RCLCPP_WARN(this->get_logger(), "⚠️ 运动启动超时");
    }
}

void X30NavBridge::handleReadyRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    RCLCPP_INFO(this->get_logger(), "🚀 收到 ready 指令, 开始执行启动序列...");

    uint8_t state;
    uint8_t gait;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // ===== 第0步: 软急停恢复 =====
    // 状态图: 软急停/摔倒 → [起立/贴下] → 贴下状态
    if (state == static_cast<uint8_t>(BasicState::SOFT_ESTOP)) {
        RCLCPP_INFO(this->get_logger(), "📌 [0/5] 机器人在软急停状态, 先恢复到贴下...");
        ensureControlTakeover();
        sendGaitCommand(CMD_STAND_UP_DOWN);

        if (!waitForBasicState({BasicState::LYING_DOWN}, 5000)) {
            res->success = false;
            res->message = "Ready failed: timeout recovering from soft estop.";
            RCLCPP_ERROR(this->get_logger(), "❌ Ready 失败: 从软急停恢复超时");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ [0/5] 已恢复到贴下状态");

        // 刷新状态
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // ===== 第1步: 起立 =====
    // 状态图: 贴下 → [起立/贴下] → 正在起立 → 初始站立
    if (state == static_cast<uint8_t>(BasicState::LYING_DOWN) ||
        state == static_cast<uint8_t>(BasicState::GOING_DOWN)) {
        RCLCPP_INFO(this->get_logger(), "📌 [1/5] 起立...");
        ensureControlTakeover();
        sendGaitCommand(CMD_STAND_UP_DOWN);

        if (!waitForBasicState(
                {BasicState::INITIAL_STAND, BasicState::FORCE_STAND, BasicState::STEPPING}, 8000)) {
            res->success = false;
            res->message = "Ready failed: timeout at step 1 (stand up).";
            RCLCPP_ERROR(this->get_logger(), "❌ Ready 失败: 起立超时");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ [1/5] 起立完成");
    } else if (state == static_cast<uint8_t>(BasicState::INITIAL_STAND) ||
               state == static_cast<uint8_t>(BasicState::FORCE_STAND) ||
               state == static_cast<uint8_t>(BasicState::STEPPING) ||
               state == static_cast<uint8_t>(BasicState::RL_MODE)) {
        RCLCPP_INFO(this->get_logger(), "✅ [1/5] 已站立, 跳过");
    } else {
        res->success = false;
        res->message = "Ready failed: unexpected state=" + std::to_string(state);
        RCLCPP_ERROR(this->get_logger(), "❌ Ready 失败: 无法识别的状态=%d", state);
        return;
    }

    // 刷新状态
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
    }

    // ===== 第2步: 力控站立 =====
    // 状态图: 初始站立 → [力控模式] → 力控站立
    if (state == static_cast<uint8_t>(BasicState::INITIAL_STAND)) {
        RCLCPP_INFO(this->get_logger(), "📌 [2/5] 切入力控站立...");
        ensureControlTakeover();
        sendGaitCommand(CMD_FORCE_CONTROL);

        if (!waitForBasicState({BasicState::FORCE_STAND}, 5000)) {
            res->success = false;
            res->message = "Ready failed: timeout at step 2 (force stand).";
            RCLCPP_ERROR(this->get_logger(), "❌ Ready 失败: 力控站立超时");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ [2/5] 力控站立完成");
    } else if (state == static_cast<uint8_t>(BasicState::FORCE_STAND) ||
               state == static_cast<uint8_t>(BasicState::RL_MODE) ||
               state == static_cast<uint8_t>(BasicState::STEPPING)) {
        RCLCPP_INFO(this->get_logger(), "✅ [2/5] 已在力控/运动, 跳过");
    } else {
        res->success = false;
        res->message =
            "Ready failed: unexpected state for force stand (state=" + std::to_string(state) + ").";
        RCLCPP_ERROR(this->get_logger(), "❌ Ready 失败: 无法切入力控(state=%d)", state);
        return;
    }

    // ===== 第3步: 切换山地步态 → 进入 RL模式 =====
    // 遥控器实际流程: 力控站立 → [山地步态指令] → RL模式 (gait=MOUNTAIN)
    // RL模式 就是山地步态下的运动状态, 不需要额外的 CMD_MOTION!
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_basic_state_;
        gait  = current_gait_state_;
    }

    // 已经在 RL模式+山地步态, 跳过
    if (state == static_cast<uint8_t>(BasicState::RL_MODE) &&
        gait == static_cast<uint8_t>(GaitState::MOUNTAIN)) {
        RCLCPP_INFO(this->get_logger(), "✅ [3/4] 已在RL模式+山地步态, 跳过");
    } else {
        RCLCPP_INFO(this->get_logger(), "📌 [3/4] 切换山地步态 (state=%d, gait=%d)...", state,
                    gait);
        ensureControlTakeover();
        sendGaitCommand(CMD_GAIT_MOUNTAIN);

        // 等待步态变为 MOUNTAIN
        if (!waitForGaitState({GaitState::MOUNTAIN}, 5000)) {
            res->success = false;
            res->message = "Ready failed: timeout at step 3 (mountain gait).";
            RCLCPP_ERROR(this->get_logger(), "❌ Ready 失败: 山地步态切换超时");
            return;
        }

        // 等待基本状态进入 RL_MODE
        if (!waitForBasicState({BasicState::RL_MODE}, 3000)) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 步态已切换但未进入RL模式, 可能仍可操作");
        }

        RCLCPP_INFO(this->get_logger(), "✅ [3/4] 山地步态+RL模式 就绪");
    }

    res->success = true;
    res->message = "Robot is ready: standing, force control, mountain gait (RL mode).";
    RCLCPP_INFO(this->get_logger(), "🎉 Ready 序列完成! 机器人已就绪 (RL模式)");
}

// ============================================================================
// 基类回调
// ============================================================================

void NavBridgeBase::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_vx_.store(msg->linear.x);
    target_vy_.store(msg->linear.y);
    target_vyaw_.store(msg->angular.z);

    // 更新最后一次收到有效指令的时间
    last_active_time_.store(std::chrono::steady_clock::now());
}

NavBridgeBase::NavBridgeBase(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options) {}

}  // namespace nav_bridge
