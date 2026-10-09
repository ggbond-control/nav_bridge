# Uniubi Robot Msgs

[English](README.md) | **简体中文**

Uniubi 机器人协议定义仓库，维护 DDS IDL、ROS 2 msg/srv 和 schema 的统一源头。

## 机器人版本要求

机器人软件版本要求：Cyvet-V1.00.000 及以上。

## 目录结构

```
.
├── idl/
│   ├── EventMessage.idl
│   ├── MotionObserved.idl
│   ├── MotorState.idl
│   ├── RPCMessage.idl
│   ├── RemoteControl.idl
│   ├── Request.idl
│   └── SensorObserved.idl
├── ros2/
│   ├── msg/
│   ├── srv/
│   │   └── System.srv
│   ├── CMakeLists.txt
│   └── package.xml
├── schema/
└── tests/
```

## 协议规则

- `idl/` 是协议唯一信源。
- `ros2/msg` 和 `ros2/srv` 由 IDL 映射生成，字段名按 ROS 2 规范使用 `snake_case`。
- 下游仓库不复制消息定义，应通过安装本仓库的 ROS 2 package 或引用发布版本获取接口。
- 协议与 SDK POD 结构的边界见 [`docs/protocol_notes.zh-CN.md`](docs/protocol_notes.zh-CN.md)。
- ROS 2 示例接入和二次开发入口见 [`uniubi_ros2`](https://github.com/uniubi-ai/uniubi_ros2/blob/main/README.zh-CN.md)。

## ROS 2 构建

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/uniubi-ai/uniubi_robot_msgs.git ~/uniubi_robot_msgs
cp -r ~/uniubi_robot_msgs ~/ros2_ws/src/uniubi_robot_msgs
cd ~/ros2_ws
colcon build --packages-select uniubi
. install/setup.bash
```

构建完成后可检查接口：

```bash
ros2 interface show uniubi/srv/System
ros2 interface show uniubi/msg/MotionObserved
ros2 interface show uniubi/msg/MotionOdometry
ros2 interface show uniubi/msg/SensorObserved
```

## IDL 与 ROS 2 字段映射

| IDL 字段 / 类型 | ROS 2 字段 / 类型 | 说明 |
|---|---|---|
| `Header.clientId` / `requestId` | `Header.client_id` / `request_id` | `Request.idl` 的 Header 字段映射；`System.srv` 不含该 Header 字段 |
| `System_Request_` / `System_Response_` | `System.srv` | ROS 2 service 合并定义 |
| `System_Request_.device_id` / `System_Response_.device_id` | `System.srv` 的 `device_id` | 目标设备 / 响应设备 SN |
| `RemoteControl_.stickLX` | `RemoteControl.stick_l_x` | 遥控器摇杆字段 |
| `MotorHeader.limbsNo` / `jointNo` | `MotorHeader.limbs_no` / `joint_no` | 电机身份字段 |
| `SensorObserved_.odom` | `SensorObserved.odom` | GPS、UWB 与 Walk 平面里程计统一观测字段 |
| `MotionOdometry.yawSpeed` | `MotionOdometry.yaw_speed` | Walk 平面里程计字段 |

`MotionOdometry` 作为 `SensorObserved.odom` 的嵌套类型，仅在 Walk 模式下有效。退出 Walk 时保留当前区间末值并将 `valid` 置为 false；再次进入 Walk 时建立新原点并递增 `epoch`。`position[2]` 和 `velocity[2]` 是三维兼容保留字段，当前固定为 `0`。

完整协议和 DDS / ROS 2 wire contract 见 [`uniubi-docs`](https://github.com/uniubi-ai/uniubi-docs/blob/main/README.zh-CN.md) 的 [`docs/uniubi_robot_dds_api.zh-CN.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.zh-CN.md)。

## 许可证

本仓库中的 UniUbi 原创 IDL、ROS 2 接口定义、schema、代码和文档使用 Apache License 2.0。详见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。

`idl/BrainMotionState.idl` 是设备已有内部 DDS 类型的原样定义，用于大脑本地观测适配。
它保留 `uniubi::dds_::BrainMotionState` 类型名，不生成同名 ROS `.msg`。ROS2 包安装时会同时安装 `idl/`，因此应保留完整仓库目录结构。
