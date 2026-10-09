# Uniubi Robot Msgs

**English** | [简体中文](README.zh-CN.md)

This repository is the single source of truth for Uniubi robot protocol definitions, including DDS IDL, ROS 2 messages and services, and schemas.

## Robot Version Requirement

Required robot software version: Cyvet-V1.00.000 or newer.

## Repository layout

```text
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

## Protocol rules

- `idl/` is the sole source of truth for the protocol.
- `ros2/msg` and `ros2/srv` are generated from the IDL mappings. Field names use ROS 2 `snake_case` conventions.
- Downstream repositories must not copy message definitions. Install this repository's ROS 2 package or depend on a published release instead.
- See [Protocol notes](docs/protocol_notes.md) for the boundary between the protocol and SDK POD structures.
- See [`uniubi_ros2`](https://github.com/uniubi-ai/uniubi_ros2) for ROS 2 examples and integration guidance.

## Build the ROS 2 package

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/uniubi-ai/uniubi_robot_msgs.git ~/uniubi_robot_msgs
cp -r ~/uniubi_robot_msgs ~/ros2_ws/src/uniubi_robot_msgs
cd ~/ros2_ws
colcon build --packages-select uniubi
. install/setup.bash
```

After the build, inspect the interfaces with:

```bash
ros2 interface show uniubi/srv/System
ros2 interface show uniubi/msg/MotionObserved
ros2 interface show uniubi/msg/MotionOdometry
ros2 interface show uniubi/msg/SensorObserved
```

## IDL-to-ROS 2 field mappings

| IDL field / type | ROS 2 field / type | Description |
|---|---|---|
| `Header.clientId` / `requestId` | `Header.client_id` / `request_id` | Header fields from `Request.idl`; `System.srv` does not contain this Header |
| `System_Request_` / `System_Response_` | `System.srv` | Combined ROS 2 service definition |
| `System_Request_.device_id` / `System_Response_.device_id` | `device_id` in `System.srv` | Target device / responding device SN |
| `RemoteControl_.stickLX` | `RemoteControl.stick_l_x` | Remote-controller stick field |
| `MotorHeader.limbsNo` / `jointNo` | `MotorHeader.limbs_no` / `joint_no` | Motor identity fields |
| `SensorObserved_.odom` | `SensorObserved.odom` | Unified GPS, UWB, and Walk planar-odometry observation |
| `MotionOdometry.yawSpeed` | `MotionOdometry.yaw_speed` | Walk planar-odometry field |

`MotionOdometry` is nested in `SensorObserved.odom` and is valid only in Walk mode. When Walk mode ends, the final value for that interval is retained and `valid` becomes false. Entering Walk mode again establishes a new origin and increments `epoch`. `position[2]` and `velocity[2]` are reserved for three-dimensional compatibility and are currently always `0`.

For the complete protocol and DDS / ROS 2 wire contract, see [`docs/uniubi_robot_dds_api.md`](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/uniubi_robot_dds_api.md) in [`uniubi-docs`](https://github.com/uniubi-ai/uniubi-docs).

## License

Uniubi-authored IDL, ROS 2 interface definitions, schemas, code, and documentation in this repository are licensed under the Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).

`idl/BrainMotionState.idl` mirrors the existing internal device DDS type for brain-local observation adapters.
It retains the `uniubi::dds_::BrainMotionState` type name and is not exposed as an identically named ROS `.msg`. Keep the complete repository layout: the ROS2 package also installs `idl/`.
