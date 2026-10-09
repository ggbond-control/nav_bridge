# 灵猫第一阶段验收记录（2026-10-08）

第一阶段通过：RK3588 与目标机器人的有线通信、DDS 发现及只读 RPC 已验证。
本阶段未申请控制权、未发送动作/速度，也未请求开启观测推送。

## 设备与网络

| 项目 | 已确认值 |
|---|---|
| 目标 SN / deviceNo | `E03C1CBAD8D2A104`，RPC 响应身份匹配 |
| 固件 / 大脑版本 | `Cyvet-V1.00.000`（用户提供） |
| MCU | `V1.0.14`（用户提供） |
| 电池固件 / 硬件 | `V1.0.1` / `V1.0.0`（用户提供） |
| 机器人有线 | `169.254.216.210/16`，MAC `e0:3c:1c:ba:d8:d2` |
| 机器人 Wi-Fi | `10.0.40.244` |
| RK3588 有线 | `eth1`，`169.254.216.1/16` |
| RK3588 管理入口 | Wi-Fi `10.0.40.163` |
| 机器人接入 | 大脑对外 `eth0`，Host Domain `42`，`/robotServer` |

系统状态返回机器人大脑与小脑的 UART、交换链路均 linked；有线接口名称为 eth0。
网络侧初期看到设备 DHCP 请求但无地址分配；后来按设备 App 提供的链路本地地址接入。
没有修改机器人网络，没有在共享交换机上启动 DHCP。

NetworkManager 持久化连接 `cyvet-link`：manual、autoconnect=yes、
`ipv4.never-default=yes`、IPv6 disabled。默认路由保持 Wi-Fi。
已检查实际路由和持久化配置，未重启开发板验证。
原 `有线连接 2` 的 DHCP 配置保留；回滚可执行：

```bash
sudo nmcli connection down cyvet-link
sudo nmcli connection up '有线连接 2'
```

## 验证结果与后续适配依据

- 最新有线 ping：30/30 成功，0% 丢包，平均 2.452 ms，最大 7.627 ms。
- 10 轮共 40 次只读 RPC 全部成功：`getMotionCapabilities`、`getSystemStatus`、`queryMotionState`、`getMotorLayout`。
- 初次建立 DDS 客户端时曾发生首请求超时；等待反向 response endpoint 匹配 500 ms 后复测成功。生产后端仍需超时及重试，不能将固定等待视为保证。
- 当前电量 70%，电压约 46.53 V，电池 online；`motionOwner=0`。
- walking 参数范围：X ±1.5 m/s、Y ±1.0 m/s、yaw ±2.5 rad/s；设备能力返回数组，不能只支持 `{actions: [...]}` 形式。
- 实际布局为 12 个关节，按 limbNo/jointNo/name 返回；完整数据已保存。
- `queryMotionState` 当前返回 `result=true` 且无 params，表示无可查询的活动动作，不应当成解析错误。

被动订阅 10 秒（未开启任何上报）：

| 通道 | 实际 publisher QoS | 样本与限制 |
|---|---|---|
| `/motion/observed` | BEST_EFFORT / VOLATILE / depth 1 | 493 帧，约 49 Hz；电量有效；accel/gyro error=0，但未完成数值及坐标验证；motor_num=0，不能发布有效关节状态 |
| `/sensor/observed` | BEST_EFFORT / VOLATILE / depth 1 | 已发现 publisher，0 帧；不能认定里程计已可用 |
| `/robotServer/Event` | RELIABLE / VOLATILE / depth 10 | 1 帧；事件 magic=0x53425645 |

事件 publisher 的实际可靠性与文档所列 BEST_EFFORT 不同。后续订阅可使用
BEST_EFFORT 以兼容两者；原始 topic 无设备身份过滤字段，联调域中需避免其他机器人的数据混入。
本阶段不包含动作、速度方向、Walk 里程计或关节数据验收。

证据文件位于 `validation/`：

- `phase1_summary_20261008.json`：验收摘要、能力、布局和数据限制。
- `phase1_rpc_20261008.jsonl`：40 次只读请求原始响应。
- `phase1_channels_20261008.jsonl`：观测通道 QoS、数量及最后一帧摘要。
- `phase1_network_20261008.txt`：地址、路由、NetworkManager 配置、RMW 版本。
- `network_probe_20261008.jsonl`：较早的 3 轮成功验证。

## 复现

开发板独立验证工作区：`/home/cat/Workspace/uniubi_network_ws`。
仅构建官方 `uniubi` 消息包，未启动 nav_bridge 或运动客户端。
消息来源 commit：`0716257e262c4b3f5d8424c6ddf284309fcac7fc`。

```bash
source /opt/ros/jazzy/setup.bash
source /home/cat/Workspace/uniubi_network_ws/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/cat/Workspace/uniubi_network_ws/cyvet_cyclonedds.xml
python3 /home/cat/Workspace/uniubi_network_ws/network_probe.py --device-id E03C1CBAD8D2A104 --repeat 10
python3 /home/cat/Workspace/uniubi_network_ws/observe_channels.py --seconds 10
```

本地对应脚本为 `network_probe.py`、`observe_channels.py`；Cyclone DDS 配置在
`../config/cyvet_cyclonedds.xml`，仅绑定机器人域 42 的 eth1。
