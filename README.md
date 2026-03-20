# Nav Bridge — ROS2 ↔ 机器狗 UDP 导航桥接节点

`nav_bridge` 是一个 ROS2 C++ 节点，充当**导航栈与机器狗底层之间的桥梁**。它将导航系统的 `/cmd_vel` 速度指令转换为机器狗理解的 UDP 协议报文（模拟手柄轴指令），同时将机器狗上报的 IMU、足式里程计等传感器数据转换为标准 ROS2 话题。当前适配绝影 X30 UDP 协议（规格书 V1.0.5）。

## 架构概览

```
导航栈 (Nav2 / Local Planner / SLAM)
        │ /cmd_vel                    ▲ /imu/data, /leg_odom
        ▼                             │
┌─────────────────────────────────────────────────┐
│              nav_bridge_node                    │
│  ┌───────────────────────────────────────────┐  │
│  │         X30NavBridge (子类)                │  │  ← 适配 X30 UDP 协议
│  │  ┌─────────────┬─────────────────────┐    │  │
│  │  │x30_protocol │ x30_nav_bridge.cpp  │    │  │     协议定义 + 业务逻辑
│  │  └─────────────┴─────────────────────┘    │  │
│  ├───────────────────────────────────────────┤  │
│  │         NavBridgeBase (基类)              │  │  ← 协议无关的 ROS2 接口
│  ├───────────────────────────────────────────┤  │
│  │         UdpTransport (通信层)             │  │  ← 纯 BSD Socket 封装
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
        │ 轴指令 UDP                  ▲ IMU/Odom/State UDP
        ▼                             │
   103 运动主机 (192.168.1.103:43893)
```

## 数据流

### 下行：ROS2 → 机器狗

```
/cmd_vel (Twist)
  → cmdVelCallback()        缓存 target_vx/vy/vyaw
  → sendCmdVelTick() @50Hz  定时器触发
    → sendVelocityCommand()
      → 查当前步态速度上限表
      → velocityToAxisValue()  映射为 [-32767, 32767]
      → 发送 3 条 CommandHead (0x21010130/131/135) 给 103
```

使用 `0x21` 前缀模拟手柄摇杆信号。在非手动+辅助模式下，感知主机会对这些信号进行碰撞检测修正。

### 上行：机器狗 → ROS2

```
103 运动主机 UDP @200Hz
  → receiveLoop() (独立线程, poll 50ms 超时)
    → 0x1008 RcsData           → 连接状态、错误报警
    → 0x1009 MotionStateData   → /leg_odom + /robot_basic_state + /robot_gait_state + TF
    → 0x100A ControllerSensor  → /imu/data  (度→弧度转换)
    → 0x21050F0A Battery       → /battery/level
```

## ROS2 话题接口

| 方向 | 话题名 | 消息类型 | 说明 |
|------|--------|---------|------|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 速度指令输入 |
| 发布 | `/imu/data` | `sensor_msgs/Imu` | IMU 数据 (200Hz) |
| 发布 | `/leg_odom` | `nav_msgs/Odometry` | 足式里程计 (200Hz) |
| 发布 | `/robot_basic_state` | `std_msgs/Int32` | 基本状态码 |
| 发布 | `/robot_gait_state` | `std_msgs/Int32` | 当前步态码 |
| 发布 | `/battery/level` | `std_msgs/UInt8` | 电池电量 % |
| TF | `odom → base_link` | — | 可通过参数关闭 |

## 编译 & 运行

```bash
# 编译
colcon build --packages-select nav_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev --symlink-install

# 运行
source install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py
# 或
ros2 run nav_bridge nav_bridge_node --ros-args --params-file src/nav_bridge/config/x30_params.yaml
```

## 配置参数

参见 `config/x30_params.yaml`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `motion_host_ip` | `192.168.1.103` | 运动主机 IP |
| `motion_host_port` | `43893` | 运动主机端口 |
| `local_recv_port` | `43897` | 本地绑定接收端口 |
| `heartbeat_interval_ms` | `200` | 心跳间隔 (≥2Hz) |
| `cmd_vel_rate_hz` | `50` | 轴指令下发频率 |
| `publish_tf` | `true` | 是否发布 odom→base_link TF |

## 文件结构

```
nav_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── x30_params.yaml            # 参数配置
├── launch/
│   └── nav_bridge.launch.py       # 启动文件
├── include/nav_bridge/
│   ├── nav_bridge_base.hpp        # 抽象基类（协议无关）
│   ├── udp_transport.hpp          # UDP socket 封装
│   ├── x30_protocol.hpp           # X30 协议定义（命令码+结构体+速度表）
│   └── x30_nav_bridge.hpp         # X30 子类头文件
└── src/
    ├── nav_bridge_node.cpp        # 节点入口
    ├── udp_transport.cpp          # UDP 实现
    └── x30_nav_bridge.cpp         # X30 实现（心跳/速度/IMU/Odom）
```

## 扩展新协议

要支持新的机器狗（如其他品牌或 CAN 总线协议）：

1. 创建 `xxx_protocol.hpp` — 定义新协议的报文结构
2. 创建 `XxxNavBridge` 继承 `NavBridgeBase`
3. 实现 `initialize()` / `shutdown()` / `sendVelocityCommand()` / `sendHeartbeat()` / `processIncomingData()`
4. 在 `nav_bridge_node.cpp` 中通过参数选择实例化哪个子类
