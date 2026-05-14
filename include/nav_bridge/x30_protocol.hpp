#pragma once

/// @file x30_protocol.hpp
/// @brief 绝影X30 UDP协议定义 — 基于《接口规格书 V1.0.5》

#include <cstdint>
#include <cstring>
#include <cmath>

// 协议结构体使用匿名struct/union以匹配官方C接口定义

namespace nav_bridge {
namespace x30_protocol {

// ============================================================================
// 协议基础结构体
// ============================================================================

/// 简单指令 (12 bytes, little-endian)
struct CommandHead {
    uint32_t code;            ///< 指令码
    uint32_t paramters_size;  ///< 指令值 (简单指令) 或 数据长度 (复杂指令)
    uint32_t type;            ///< 0=简单指令, 1=复杂指令
} __attribute__((packed));

static_assert(sizeof(CommandHead) == 12, "CommandHead must be 12 bytes");

/// 复杂指令接收缓冲
struct CommandMessage {
    CommandHead head;
    uint8_t data_buffer[1024];
} __attribute__((packed));

// ============================================================================
// 网络地址
// ============================================================================

constexpr char MOTION_HOST_IP[] = "192.168.1.103";  ///< 运动主机
constexpr int MOTION_HOST_PORT  = 43893;

constexpr char PERCEPT_HOST_IP[] = "192.168.1.105";  ///< 感知主机
constexpr int PERCEPT_HOST_PORT  = 43899;
constexpr int PERCEPT_LIDAR_PORT = 60000;
constexpr int PERCEPT_CHARGE_PORT = 3333;
constexpr int PERCEPT_CHARGE_LOCAL_PORT = 49004;

// ============================================================================
// 发送指令码 (0x21 前缀 = 模拟手柄/遥控器, 0x31 前缀 = 导航模块)
// 本实现使用 0x21 前缀以模拟手柄摇杆信号
// ============================================================================

// --- 心跳 & 查询 ---
constexpr uint32_t CMD_HEARTBEAT = 0x21040001;
constexpr uint32_t CMD_QUERY_103 = 0x21020001;  ///< 确认连接

// --- 基本状态转换 ---
constexpr uint32_t CMD_STAND_UP_DOWN = 0x21010202;  ///< 起立/趴下切换
constexpr uint32_t CMD_FORCE_CONTROL = 0x2101020A;  ///< 力控模式
constexpr uint32_t CMD_MOTION        = 0x21010201;  ///< 开始/停止运动

// --- 轴指令 (速度控制), 值域 [-32767, 32767], 下发频率 50Hz ---
constexpr uint32_t CMD_VEL_FORWARD = 0x21010130;  ///< 左摇杆Y: 前后 (正前负后)
constexpr uint32_t CMD_VEL_LATERAL = 0x21010131;  ///< 左摇杆X: 左右 (正右负左)
constexpr uint32_t CMD_VEL_YAW     = 0x21010135;  ///< 右摇杆X: 转向 (正右负左)
constexpr uint32_t CMD_BODY_HEIGHT = 0x21010102;  ///< 右摇杆Y: 身体高度

// --- 控制模式 ---
constexpr uint32_t CMD_MANUAL_MODE     = 0x21010C02;  ///< 切入手动模式
constexpr uint32_t CMD_NON_MANUAL_MODE = 0x21010C03;  ///< 切入非手动模式

// --- 身体高度 ---
constexpr uint32_t CMD_HEIGHT_SWITCH = 0x21010406;  ///< 值: 0=匍匐, 2=正常

// --- 步态 ---
constexpr uint32_t CMD_GAIT_WALK        = 0x21010300;  ///< 行走步态
constexpr uint32_t CMD_GAIT_SLOPE       = 0x21010402;  ///< 斜坡步态
constexpr uint32_t CMD_GAIT_OBSTACLE    = 0x21010401;  ///< 越障步态
constexpr uint32_t CMD_GAIT_STAIR       = 0x21010405;  ///< 楼梯步态
constexpr uint32_t CMD_GAIT_STAIR_ACC   = 0x2101040A;  ///< 楼梯步态(累积帧)
constexpr uint32_t CMD_GAIT_STAIR45_ACC = 0x2101040B;  ///< 45°楼梯(累积帧)
constexpr uint32_t CMD_GAIT_MOUNTAIN    = 0x21010421;  ///< 山地步态
constexpr uint32_t CMD_GAIT_L_WALK      = 0x21010420;  ///< L行走步态
constexpr uint32_t CMD_GAIT_SILENT      = 0x21010422;  ///< 静音步态
constexpr uint32_t CMD_GAIT_RUN         = 0x31010307;  ///< 跑步步态

// --- 其他 ---
constexpr uint32_t CMD_SOFT_ESTOP = 0x21010C0E;  ///< 软急停
constexpr uint32_t CMD_SAVE_DATA  = 0x010C01;    ///< 保存数据

// --- 感知主机 (发往 105:43899) ---
constexpr uint32_t CMD_VEL_SOURCE = 0x3101EE03;  ///< 速度输入源: 1=手柄, 2=导航
constexpr uint32_t CMD_GROUND_MODE =
    0x3101EE01;  ///< 地形图模式: 3=实心, 4=镂空, 5=无踢面, 20=累积帧
constexpr uint32_t CMD_BRAKE_MODE      = 0x3101EE02;  ///< 停避障模式: 1=停障, 2=避障
constexpr uint32_t CMD_OBSTACLE_HEIGHT = 0x3101EE04;  ///< 障碍高度: 1=8cm, 2=28cm
constexpr uint32_t CMD_CHARGE_MANAGER = 0x91910250;   ///< 自主充电请求
constexpr uint32_t CMD_CHARGE_MANAGER_QUERY = 0x91910253;  ///< 自主充电查询
constexpr int32_t CHARGE_COMMAND_START_VALUE = 1;
constexpr int32_t CHARGE_COMMAND_STOP_VALUE = 0;
constexpr int32_t CHARGE_COMMAND_RESET_VALUE = 2;
constexpr int32_t CHARGE_COMMAND_QUERY_VALUE = 0;

// --- 感知主机 (发往 105:60000) ---
constexpr uint32_t CMD_LIDAR_ODOM = 0x0BAA0001;  ///< 激光里程计: 1=开, 0=关

// ============================================================================
// 接收指令码
// ============================================================================

constexpr uint32_t RECV_RCS_DATA          = 0x1008;      ///< 运行状态, 200Hz
constexpr uint32_t RECV_MOTION_STATE      = 0x1009;      ///< 运动状态, 200Hz
constexpr uint32_t RECV_CONTROLLER_SENSOR = 0x100A;      ///< 传感器数据, 200Hz
constexpr uint32_t RECV_CONTROLLER_SAFE   = 0x100B;      ///< 系统状态, 1Hz
constexpr uint32_t RECV_BATTERY           = 0x21050F0A;  ///< 电池信息, 0.5Hz
constexpr uint32_t RECV_BODY_HEIGHT       = 0x11050F08;  ///< 身体高度状态

// 感知主机反馈
constexpr uint32_t RECV_VEL_SOURCE_STATE  = 0x3100EE03;  ///< 速度输入源状态
constexpr uint32_t RECV_GROUND_MODE_STATE = 0x3100EE01;  ///< 地形图模式状态
constexpr uint32_t RECV_BRAKE_MODE_STATE  = 0x3100EE02;  ///< 停避障模式状态

// ============================================================================
// 轴指令参数
// ============================================================================

constexpr int32_t AXIS_VALUE_MAX    = 32767;
constexpr int32_t AXIS_VALUE_MIN    = -32767;
constexpr int32_t AXIS_DEAD_ZONE    = 655;  ///< 死区: |value| < 655 时视为0
constexpr int CMD_VEL_RATE_HZ       = 50;   ///< 轴指令下发频率
constexpr int HEARTBEAT_INTERVAL_MS = 200;  ///< 心跳间隔 (≥2Hz)

// ============================================================================
// 接收数据结构体
// ============================================================================

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

/// IMU 传感器数据
struct ImuSensorData {
    int32_t timestamp;
    union {
        float buffer_float[9];
        uint8_t buffer_byte[3][12];
        struct {
            float roll, pitch, yaw;           ///< 角度 (°)
            float omega_x, omega_y, omega_z;  ///< 角速度 (rad/s)
            float acc_x, acc_y, acc_z;        ///< 加速度 (m/s²)
        };
    };
};

/// 腿部关节数据
struct LegJointData {
    union {
        float data[12];
        struct {
            float fl_hipx, fl_hipy, fl_knee;  ///< 左前
            float fr_hipx, fr_hipy, fr_knee;  ///< 右前
            float hl_hipx, hl_hipy, hl_knee;  ///< 左后
            float hr_hipx, hr_hipy, hr_knee;  ///< 右后
        };
    };
};

/// 运动控制传感器数据 (指令码 0x100A, 200Hz)
struct ControllerSensorData {
    ImuSensorData imu_data;
    LegJointData joint_pos;  ///< 关节位置 (rad)
    LegJointData joint_vel;  ///< 关节速度 (rad/s)
    LegJointData joint_tau;  ///< 关节力矩 (N·m)
};

/// 运动状态数据 (指令码 0x1009, 200Hz)
/// 注意: 不使用 packed, 与运动主机自然对齐一致
/// uint8_t basic/gait 后有 2 字节 padding 对齐 float
struct MotionStateData {
    uint8_t basic_state;     ///< 基本状态 [4]
    uint8_t gait_state;      ///< 步态 [5]
    float max_forward_vel;   ///< 已弃用
    float max_backward_vel;  ///< 已弃用
    float leg_odom_pos[3];   ///< 世界坐标系位姿 {x(m), y(m), yaw(rad)}
    float leg_odom_vel[3];   ///< 身体坐标系速度 {vx(m/s), vy(m/s), vyaw(rad/s)}
    float robot_distance;    ///< 当次开机运行里程 (cm)
    unsigned touch_state;    ///< 预留
    union {
        struct {
            uint32_t narrow_walk : 1;
            uint32_t pose_safe_flag : 1;
            uint32_t joint_limit_flag : 1;
            uint32_t state_reserved : 29;
        } control_state_bit;
        uint32_t control_state;
    };
    union {
        struct {
            uint8_t auto_charge_state;
            uint8_t pos_ctrl_state;
            uint8_t task_reserved[8];
        } task_state_list;
        uint8_t task_state[10];
    };
};

/// 机器人运行状态 (指令码 0x1008, 200Hz)
/// 注意: 使用自然对齐, long 在 x86_64 上为 8 字节, 此处用 int64_t 显式表示
struct RcsData {
    char robot_name[15];
    int32_t current_mileage;      ///< 当次里程 (cm)
    int32_t total_mileage;        ///< 累计里程 (cm)
    int64_t current_run_time;     ///< 当次运行时间 (s)
    int64_t total_run_time;       ///< 累计运行时间 (s)
    int64_t current_motion_time;  ///< 当次运动时间 (s)
    int64_t total_motion_time;    ///< 累计运动时间 (s)
    float joystick[4];            ///< 轴指令 [LX, LY, RX, RY] 范围[-1,1]
    union {
        struct {
            uint8_t is_nav_mode;  ///< 0=手动, 1=非手动
            uint8_t emergency_source;
            uint8_t mode_reserved[8];
        } rcs_state_list;
        uint8_t rcs_state[10];
    };
    union {
        struct {
            uint32_t imu_error : 1;
            uint32_t wifi_error : 1;
            uint32_t driver_heat_warn : 1;
            uint32_t driver_error : 1;
            uint32_t motor_heat_warn : 1;
            uint32_t battery_low_warn : 1;
            uint32_t battery_heat_warn : 1;
            uint32_t gpio_error : 1;
            uint32_t cpu_heat_warn : 1;
            uint32_t cpu_freq_warn : 1;
            uint32_t reserved : 22;
        } error_state_bit;
        uint32_t error_state;
    };
};

/// 电池信息 (指令码 0x21050F0A, 0.5Hz)
struct BatterySensorData {
    uint16_t voltage;
    int16_t current;
    uint16_t remaining_capacity;
    uint16_t nominal_capacity;
    uint16_t cycles;
    uint16_t production_date;
    uint16_t balanced_low;
    uint16_t balanced_high;
    uint16_t protected_state;
    uint8_t software_version;
    uint8_t battery_level;  ///< 剩余容量百分比 (%)
    uint8_t mos_state;
    uint8_t battery_quantity;
    uint8_t battery_ntc;
    float battery_temperature[4];
} __attribute__((packed));

/// CPU 信息
struct CpuInfo {
    float temperature;  ///< ℃
    float frequency;    ///< MHz
};

/// 运动控制系统状态 (指令码 0x100B, 1Hz)
struct ControllerSafeData {
    float motor_temperature[12];    ///< 电机温度 (℃)
    uint8_t driver_temperture[12];  ///< 驱动器温度 (℃) (原文拼写)
    CpuInfo cpu_info_;
};

#pragma GCC diagnostic pop

// ============================================================================
// 基本状态枚举
// ============================================================================

enum class BasicState : uint8_t {
    LYING_DOWN    = 0,   ///< 趴下
    STANDING_UP   = 1,   ///< 正在起立
    INITIAL_STAND = 2,   ///< 初始站立
    FORCE_STAND   = 3,   ///< 力控站立
    STEPPING      = 4,   ///< 踏步（运动中）
    GOING_DOWN    = 5,   ///< 正在趴下
    SOFT_ESTOP    = 6,   ///< 软急停/摔倒
    RL_MODE       = 16,  ///< RL状态
};

enum class GaitState : uint8_t {
    WALK        = 0,
    OBSTACLE    = 1,
    SLOPE       = 2,
    RUN         = 3,
    STAIR_SOLID = 6,  ///< 楼梯(实心/镂空/无踢面)
    STAIR_ACC   = 7,  ///< 楼梯(累积帧)
    STAIR45_ACC = 8,  ///< 45°楼梯(累积帧)
    L_WALK      = 32,
    MOUNTAIN    = 33,
    SILENT      = 34,
};

// ============================================================================
// 各步态最大速度表 (正常高度)
// ============================================================================

struct GaitSpeedLimit {
    float max_forward;   ///< m/s
    float max_backward;  ///< m/s
    float max_lateral;   ///< m/s
    float max_yaw;       ///< rad/s
};

/// 获取步态对应的速度上限
inline GaitSpeedLimit getGaitSpeedLimit(uint8_t gait) {
    switch (static_cast<GaitState>(gait)) {
        case GaitState::WALK:
            return {1.2f, 0.7f, 0.75f, 0.5f};
        case GaitState::OBSTACLE:
            return {0.3f, 0.3f, 0.15f, 0.45f};
        case GaitState::SLOPE:
            return {0.3f, 0.3f, 0.1f, 0.5f};
        case GaitState::STAIR_SOLID:
            return {0.3f, 0.3f, 0.2f, 0.8f};
        case GaitState::STAIR_ACC:
            return {0.3f, 0.3f, 0.2f, 0.8f};
        case GaitState::STAIR45_ACC:
            return {0.3f, 0.3f, 0.2f, 0.8f};
        case GaitState::L_WALK:
            return {1.0f, 1.0f, 0.5f, 1.2f};
        case GaitState::MOUNTAIN:
            return {1.0f, 1.0f, 0.8f, 1.2f};
        case GaitState::RUN:
            return {0.6f, 0.25f, 0.4f, 0.5f};
        case GaitState::SILENT:
            return {1.0f, 1.0f, 0.8f, 1.2f};
        default:
            return {0.5f, 0.3f, 0.2f, 0.5f};
    }
}

// ============================================================================
// 辅助函数
// ============================================================================

/// 构造简单指令
inline CommandHead makeSimpleCommand(uint32_t code, int32_t value = 0) {
    CommandHead cmd;
    cmd.code = code;
    // 注意: paramters_size 字段在简单指令中存储的是指令值
    // 使用 memcpy 以安全地在 uint32_t 和 int32_t 之间转换
    std::memcpy(&cmd.paramters_size, &value, sizeof(value));
    cmd.type = 0;
    return cmd;
}

/// 将物理速度映射到轴指令值 [-32767, 32767]
/// @param velocity  物理速度 (m/s 或 rad/s)
/// @param max_vel   当前步态最大速度
/// @return 映射后的轴指令值
inline int32_t velocityToAxisValue(float velocity, float max_vel) {
    if (max_vel <= 0.0f) return 0;
    float ratio = velocity / max_vel;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;
    
    int32_t raw_val = static_cast<int32_t>(ratio * AXIS_VALUE_MAX);
    // 绝影底层通信摇杆死区过滤，避免小数值带来的怠速漂移
    if (std::abs(raw_val) < AXIS_DEAD_ZONE) {
        return 0;
    }
    return raw_val;
}

// ============================================================================
// 自主充电状态
// ============================================================================

constexpr uint16_t CHARGE_STATE_IDLE = 0x0000;
constexpr uint16_t CHARGE_STATE_DO_CHARGE_TASK = 0x0001;
constexpr uint16_t CHARGE_STATE_CHARGING = 0x0002;
constexpr uint16_t CHARGE_STATE_DO_OVER_CHARGE_TASK = 0x0003;
constexpr uint16_t CHARGE_STATE_PILE_ERROR = 0x0004;
constexpr uint16_t CHARGE_STATE_SAFETY_WARNING = 0x0005;
constexpr uint16_t CHARGE_STATE_TAG_RECV_TIMEOUT = 0x1002;
constexpr uint16_t CHARGE_STATE_MARK_LAUNCH_FAILED = 0x1003;
constexpr uint16_t CHARGE_STATE_TAG_NO_VALUE = 0x1005;
constexpr uint16_t CHARGE_STATE_GOTO_STACK_FAILED = 0x1004;
constexpr uint16_t CHARGE_STATE_TAG_POSE_JUMP = 0x1006;
constexpr uint16_t CHARGE_STATE_NO_CHARGE_PLUG = 0x1007;
constexpr uint16_t CHARGE_STATE_NO_CHARGE_PLUG_STEP_BACK = 0x100A;

inline const char *chargeStateToString(uint16_t state)
{
    switch (state)
    {
    case CHARGE_STATE_IDLE:
        return "idle";
    case CHARGE_STATE_DO_CHARGE_TASK:
        return "do_charge_task";
    case CHARGE_STATE_CHARGING:
        return "charging";
    case CHARGE_STATE_DO_OVER_CHARGE_TASK:
        return "do_over_charge_task";
    case CHARGE_STATE_PILE_ERROR:
        return "pile_error";
    case CHARGE_STATE_SAFETY_WARNING:
        return "safety_warning";
    case CHARGE_STATE_TAG_RECV_TIMEOUT:
        return "TAGRECETIMEOUT";
    case CHARGE_STATE_MARK_LAUNCH_FAILED:
        return "MARKLAUNCHFAILED";
    case CHARGE_STATE_TAG_NO_VALUE:
        return "TAGNOVALUE";
    case CHARGE_STATE_GOTO_STACK_FAILED:
        return "GOTOSTACKFAILED";
    case CHARGE_STATE_TAG_POSE_JUMP:
        return "TAGPOSEJUMP";
    case CHARGE_STATE_NO_CHARGE_PLUG:
        return "NOCHARGEPLUG";
    case CHARGE_STATE_NO_CHARGE_PLUG_STEP_BACK:
        return "NOCHARGEPLUG_STEP_BACK";
    default:
        return "unknown";
    }
}

}  // namespace x30_protocol
}  // namespace nav_bridge
