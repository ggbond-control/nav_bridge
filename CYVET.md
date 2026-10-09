# 宇泛 Cyvet 适配与维护

本文整合原 `cyvet/` 中的构建、网络、终端、模型切换、故障修复和阶段验收说明，
以及宇泛依赖的本地补丁说明。X30、D1 Max 的原文档和代码保持原有结构。

## 架构与目录

```text
现有 ROS 接口 → CyvetNavBridgeNode → CyvetBackend
             → uniubi_motion_client → robotServer
```

`include/nav_bridge/robot_backend.hpp` 是通用后端接口；Cyvet 的头文件、实现、
参数分别位于 `include/nav_bridge/cyvet`、`src/cyvet`、`config/cyvet_params.yaml`。
统一入口仍为 `launch/nav_bridge.launch.py`，选择 `robot_type: cyvet`。
宇泛消息和运动客户端固定在 `third_party/uniubi`，与 nav_bridge 构建到同一个工作区。
不依赖独立的 `cyvet_adapter_ws`、`uniubi_network_ws`，不链接宇泛预编译运动库。

`test/cyvet` 集中保存自动测试和手动实机检查。测试脚本不安装到运行包，
不会由 launch 或构建自动执行实机动作。历史 validation、哈希清单、构建包装脚本
及未使用的厂商示例已经移出源码交付；构建使用下面的标准 colcon 命令。

保留 `third_party/uniubi/COLCON_IGNORE`，使常规 X30/D1 包发现不自动引入宇泛依赖。
构建 Cyvet 时通过显式 `--base-paths` 发现两个依赖包。
厂商 `LICENSE`、`NOTICE`、完整消息/IDL 定义及实际使用的 native DDS 观测读取器均保留。

## 构建与启动

已验证环境为 Ubuntu 24.04、ROS 2 Jazzy，支持 x86_64 和 ARM64。以下命令在
工作区根目录执行；开发板工作区为 `/home/cat/Workspace/driver_ws`。
首次构建或清理旧工作区时，从仅加载系统 ROS 的新 shell 构建，避免把旧 overlay
写入 `install/setup.*` 的环境链。

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds
colcon build --base-paths src/nav_bridge/third_party/uniubi/uniubi_robot_msgs/ros2 \
  src/nav_bridge/third_party/uniubi/uniubi_motion_client \
  --packages-select uniubi uniubi_motion_client \
  --symlink-install --cmake-clean-cache --parallel-workers 2 \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
colcon build --base-paths src/nav_bridge --packages-select nav_bridge \
  --symlink-install --cmake-clean-cache \
  --cmake-args -DNAV_BRIDGE_BUILD_X30=OFF -DNAV_BRIDGE_BUILD_D1_MAX=OFF \
  -DNAV_BRIDGE_BUILD_CYVET=ON -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py robot_type:=cyvet
```

开发板源码和安装的 `config/nav_bridge.yaml` 均为 `robot_type: cyvet`，所以
`ros2 launch nav_bridge nav_bridge.launch.py` 可以直接启动。仓库通用默认仍为 x30；
安装不会隐藏改写型号。launch 可接受 `params_file:=/absolute/params.yaml`，
以及 `cyclonedds_config:=/absolute/cyclonedds.xml`。

开发板 `.zshrc` 加载现有 driver_ws/algor_ws/task_ws，业务域为 81，RMW 为
`rmw_cyclonedds_cpp`，XML 为 `/home/cat/Config/cyclonedds_large_message.xml`。
launch 优先继承终端 XML；没有环境配置时使用包内 `config/cyvet_cyclonedds.xml`。
机器人独立 context 使用 Domain 42，业务节点仍沿用调用者的域。

## 网络与设备

| 项目 | 当前部署值 |
|---|---|
| device_id / SN | E03C1CBAD8D2A104 |
| 固件 / 大脑 | Cyvet-V1.00.000 |
| MCU | V1.0.14 |
| 电池固件 / 硬件 | V1.0.1 / V1.0.0 |
| 机器人有线 | 169.254.216.210/16，MAC e0:3c:1c:ba:d8:d2 |
| 机器人 Wi-Fi | 10.0.40.244 |
| RK3588 有线 | eth1，169.254.216.1/16 |
| RK3588 管理 Wi-Fi | 10.0.40.163 |
| 机器人服务与事件 | Domain 42，robotServer，/robotServer/Event |

NetworkManager 连接 `cyvet-link` 使用静态有线地址、自动连接、never-default，
保留 Wi-Fi 管理默认路由。机器人外部网口不提供 DHCP。
业务 XML 的环回/大消息配置限定于 Domain 81，机器人域 42 绑定 eth1 并开启多播。
当前业务域只能从板端访问，远程操作通过 SSH 到开发板。

DDS 还会广播机器人 Wi-Fi locator。绑定 eth1 不等于所有远端单播都从 eth1 发出，
Linux 路由仍决定发包接口。本设备已持久化下面的目标路由：

```text
10.0.40.244/32 via 169.254.216.210 dev eth1 metric 50
```

检查命令：

```bash
ip route get 10.0.40.244 from 169.254.216.1
nmcli connection show cyvet-link
```

应显示 `via 169.254.216.210 dev eth1`。若新设备需要添加这条路由，先检查是否已存在，
不要重复添加；本开发板已配置：

```bash
sudo nmcli connection modify cyvet-link +ipv4.routes '10.0.40.244/32 169.254.216.210 50'
sudo nmcli device reapply eth1
```

机器人 IP 变化时须更新路由。该跨接口访问方案只在本设备验证。
移除该路由可把 `+ipv4.routes` 改为 `-ipv4.routes` 后 reapply。
此前尝试 prefer_multicast=true 导致 RPC 全部失败，没有采用。

## 控制语义与 ROS 接口

启动只读：注册回调、连接、查询能力、开启观测，不自动申请控制权。
只有显式 `stand/ready` 才取权，并启动三轴零速 walking，确认后允许 `/cmd_vel`。
所有 SDK 调用与内部 executor 由一个工作线程串行处理，避免同步 RPC 重入。
停止请求优先，速度只缓存最新一条；失权、断线、停止、切档清空缓存。
重连只恢复观测，重新运动必须显式调用服务。

| 服务 | 类型 | Cyvet 行为 |
|---|---|---|
| stand、ready | std_srvs/srv/Trigger | 显式取权，选择默认 slow，确认零速 walking |
| lie | 同上 | 确认 laying，然后释放控制权 |
| soft_estop | 同上 | 立即封锁本地输入，请求 emergencyStop；仅 stand/ready 恢复 |
| release_control | 同上 | stopAction，确认停止后释放 |
| set_gait | rcl_interfaces/srv/SetParameters | 一项 gait，选择厂商 slow/fast 模型 |
| set_speed | 同上 | 兼容入口：整数 speed=1 对应 slow，3 对应 fast，2 不支持 |
| set_body_height、charge_command | 同上 | 明确返回不支持 |

服务均在 `/nav_bridge_node` 下。正常退出也先确认停止，再释放。
不能把释放成功当作机器人已经停止。急停生效后释放不会重新请求 walking。
停止 RPC 失败时请求 emergencyStop 并持续确认；确认前拒绝新的站立准备。

`/cmd_vel` 使用 Twist 的 linear.x、linear.y、angular.z，单位 m/s、m/s、rad/s。
拒绝 NaN/Inf，三轴完整发送。`lateral_sign=+1` 已现场校准：正 X 前进、正 Y 左移、
正 yaw 左转；原官方文字与 ROS 示例横向描述相反，因此保留符号配置。
没有适配层三档限速或速度裁剪；标准键盘的默认 0.5 m/s、1.0 rad/s 完整发送。
持续运动需要持续发布新输入；500 ms 无输入时执行停止动作并确认，不仅发送零速参数。

当前部署派发 10 Hz，速度/停止 RPC 超时 500 ms，输入超时 500 ms，连接超时 5 s，
重连间隔 2 s，动作确认超时 10 s，数据有效期 2 s，请求租约 5 s。
代码的通用默认派发为 30 Hz、RPC 超时 200 ms，实际开发板使用参数文件覆盖。
SDK 取权有约 3 s master 切换等待，release RPC 超时 5 s；停止无法中断已经发出的 RPC。

## 厂商模型切换与键盘

官方 High-level 接口仅公开字符串 controlProfile="slow" / "fast"，
没有独立 medium 模型。内部数字 profile ID 不能代替字符串参数。
沿用现有 set_gait 名称和类型，与 D1 的慢/快通用模式编号保持对应：

| set_gait 输入 | 厂商参数 | /robot_gait_state |
|---|---|---|
| WALK、SLOW 或整数 0 | controlProfile="slow" | 0 |
| RUN、FAST 或整数 3 | controlProfile="fast" | 3 |

字符串不区分大小写；MEDIUM、SILENT/34 及其他步态明确拒绝。
`default_control_profile` 默认 slow，每次 stand/ready 都显式恢复默认值，不继承上次 fast。
切档不取权或解除急停，必须已有导航就绪状态；先停止并清空旧速度，发送三轴全零的
profile，确认 walking/零速和目标档位，再恢复输入。旧命令不重放，须发送新 cmd_vel。
厂商明确拒绝、身份错误、超时、档位不匹配或并发停止都不会伪报切换成功。
每次后续速度更新完整携带当前 profile。

```bash
ros2 service call /nav_bridge_node/stand std_srvs/srv/Trigger '{}'
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

另一个终端切 fast / slow：

```bash
ros2 service call /nav_bridge_node/set_gait rcl_interfaces/srv/SetParameters \
  '{parameters: [{name: gait, value: {type: 2, integer_value: 3}}]}'
ros2 service call /nav_bridge_node/set_gait rcl_interfaces/srv/SetParameters \
  '{parameters: [{name: gait, value: {type: 2, integer_value: 0}}]}'
```

键盘 i/, 为前后，J/L 为左右横移，j/l 为左右旋转，k/空格为零速，q/z 调整输入速度。
退出键盘后调用 lie 趴下并释放，或 release_control 停止并释放。
退出桥接用 Ctrl+C，触发正常清理。

官方定义的 slow 范围为 vx [-1.5,1.5]、vy [-1,1]、yaw [-2.5,2.5]；
fast 为 vx [-2.5,3]、vy [-1,1]、yaw [-3,3]。这是官方参数范围，非实测最高速度。
本固件显式传 profile 后会回报档位名称；RPC ACK 可能早于有效状态更新，需要轮询。
部分版本不报告档位时，服务结果会明确说明只确认 RPC 接受和 walking/零速。

## 状态和传感器

| 话题 | 类型 / 说明 |
|---|---|
| /battery/level | UInt8，真实电量，未知或过期不发布，不用 0 代替未知 |
| /robot_basic_state | Int32，通用后端状态，不能按 X30 固件枚举解释 |
| /robot_gait_state | Int32，最近成功选择的 0=slow、3=fast |
| /leg_odom | Odometry，有效 Walk 里程计，epoch/重连通过 SE(2) 偏移连续衔接 |
| /imu/data | Imu，有限值且无 accel/gyro 错误；归一化四元数，无效 orientation 标未知 |
| /joint_states | JointState，按实际电机布局生成名称，完整在线无错误才发布 |
| /robot_fault | String，连接、控制和 RPC 故障 |
| /nav_bridge_node/backend_status | String JSON，连接、持权、导航就绪、停止确认、动作、缓存、错误及 profile |

基本状态为 0 UNKNOWN、1 LYING_DOWN、2 STANDING_UP、3 STANDING、4 MOVING、
5 LOCKED、6 ESTOP；walking 零速映射 3，非零速映射 4。
backend_status 中的速度是运控查询结果，非独立机身速度测量。
control_profile_selected / acknowledged 表示最近成功选择；reported 是最近固件回复值。
失权后历史档位不代表设备当前模型，必须同时查看 connected/control_owned/navigation_ready。

frame 默认 odom、base_link、imu_link，不发布 odom→base_link TF。
只有有效、新鲜、时间戳递增的样本用于里程计，不推算重连缺失期间位移。
时间戳对 ROS 的绝对对齐尚未标定，对外消息使用业务节点接收时间。
当前设备通过 native DDS BrainMotionState 读取传感器/里程计，不能把该读取器当作冗余删除。

## 超时与续租修复

抓包证实早期机器人 Wi-Fi locator 的请求走 wlan0、有线回包，专用有线路由修复了该路径。
修复后大部分 RPC 延迟约 3–30 ms，设备内部仍偶发约 5 s 后返回失败。
客户端不能宣称修复了固件内部根因。

SDK 用独立 SystemRpcTimeout / kRpcTimeout 区分真正 deadline expiry。
速度超时仅在仍持权、fresh walking、导航就绪未取消、且有比失败命令更新并未超 500 ms
的输入时，重试一次最新速度。明确拒绝、身份错误、输入过期、停止/抢权或再次失败仍停止。
不会重放失败命令、自动站立或解除急停；诊断记录 velocity_rpc_timeouts / recoveries。

续租响应等待按 min(3 s, max(200 ms, lease/10)) 限制，并不越过已确认的租约到期时间。
5 s 租约单次等待为 500 ms，留出提前重试机会；没有有效回复或控制动作不延长本地租约。
真正失权或到期仍封锁。节点将实际 fault 限频输出到终端，避免只看到准备提示。

## 测试与验收

| 文件（test/cyvet 下） | 用途 |
|---|---|
| integration.py | loopback 隔离域 174/175，协议、故障、抢权、缓存、模型切换及续租自动测试 |
| test_motion_math.cpp | Walk epoch 和重新连接的里程计连续性 |
| network_probe.py | Domain 42 只读能力、系统状态、运动状态、电机布局，不取权 |
| observe_channels.py | 被动读取观测通道、QoS 和样本，不发动作 |
| read_only_smoke.py | 启动桥接并确认只读、真实电量/IMU |
| keyboard_readonly_smoke.py | 开发板新 zsh、默认 launch 和实际键盘 Twist 链路，不取权 |
| motion_smoke.py | 现场限时姿态、六方向、组合、急停、控制权与 profile 检查 |
| readiness_smoke.py | 现有导航数据就绪→stand 流程、lie/释放，需要外部 readiness 脚本 |

厂商 `uniubi_motion_client/test/cere_sensor_reader_test.cpp` 是 native DDS 读取回归，
保留在厂商包内以便该依赖独立构建测试。
构建时把 BUILD_TESTING 改为 ON，再执行：

```bash
colcon test --packages-select nav_bridge --ctest-args -R '^cyvet_' --output-on-failure
colcon test --packages-select uniubi_motion_client --ctest-args --output-on-failure
colcon test-result --verbose
```

默认实机只读验证：

```bash
python3 src/nav_bridge/test/cyvet/read_only_smoke.py --launch \
  --params src/nav_bridge/config/cyvet_params.yaml
```

直接查机器人域时需要 ROS_DOMAIN_ID=42；业务测试沿用当前域。运动测试只在现场
可接管、区域清空时显式执行。例如已启动桥接时：

```bash
python3 src/nav_bridge/test/cyvet/motion_smoke.py --stage profiles
python3 src/nav_bridge/test/cyvet/motion_smoke.py --stage axes --seconds 3 \
  --lateral-speed 0.1 --yaw-speed 0.3
```

profiles 只检查零速 slow→fast→slow→fast、ready 默认恢复 slow、lie/释放；不会发平移或转向。
自动测试和手动运动脚本均不注册到默认 launch。

2026-10-09 阶段验收包含：x86/ARM64 构建、Cyvet OFF 时原型号构建、模拟故障、
只读网络/身份、站立/趴下、六方向现场确认、超时停止、急停及显式恢复、数据连通性。
两分钟零速加压遇一次速度超时后用最新输入恢复；90 s 空闲遇两次续租超时后提前重试恢复。
六方向各 3 s 现场确认正常；零速 profile 切换有固件 slow/fast 回报，最终趴下释放。
首次切档即时比较旧状态造成失败，修复为等待目标状态后复验通过。
这些结果不代表固件间歇失败已经消除。

尚未验收：fast 的非零运动特性/最高速度、人工遥控抢权、真实拔线后的自主停止、
短巡检路线、30 分钟导航、外参/绝对里程计精度以及重启后的完整网络复验。
断网后客户端不能保证停止指令送达，必须单独测量设备租约失效和自主停止行为。
导航路线目前只使用 WALK/0，关闭自主充电；实机导航验收完成前不配置自启动。

## 宇泛依赖维护与目录清理

来源和固定 commits 保存在 `third_party/uniubi/versions.json`。官方资料包括
uniubi-docs、uniubi_ros2、uniubi_robot_msgs，mock 只作协议参考；Python SDK、
视频音频、模型训练及机器人描述不进入当前运行依赖。

相对固定的官方来源，本地保留的补丁为：

- Event reader 使用 BEST_EFFORT/VOLATILE，兼容文档与实机 RELIABLE publisher。
- 同步 RPC、异步续租都检查响应 device_id；错误设备不能授权动作。
- deadline expiry 独立异常/错误码，原枚举数值保持，允许受限速度恢复。
- 续租响应等待按实际租约限制，不延长未被确认的租约。
- native DDS BrainMotionState 转换为 SensorObserved，读取设备实际传感器和 Walk 里程计。
- 只构建运行客户端和 native DDS 测试；删除未使用的 CLI、示例和观测演示程序。
- JsonCpp 直接编译三个 lib_json 源文件；删除其未调用的独立项目构建模板，保留源码、头文件和许可。

JsonCpp 来源为 https://github.com/open-source-parsers/jsoncpp，许可为 Public Domain 或 MIT，
完整文本位于 `uniubi_motion_client/third_party/jsoncpp/LICENSE`；说明已并入该客户端 NOTICE。
宇泛消息/IDL 及客户端的 Apache-2.0 LICENSE 和原 NOTICE 保留。

开发板旧 backups、cyvet_adapter_ws、uniubi_network_ws 只包含此前联调与回滚材料。
删除前必须检查当前 install 的符号链接、动态库和 setup 环境链都不引用旧工作区。
旧链若仍存在，从仅加载 /opt/ros/jazzy 的环境重建上述三个包，再验证新终端和只读 launch。
这些目录不属于运行依赖；网络持久化配置在 NetworkManager/netplan 和 ~/Config，
删除工作区不会代替或撤销网络配置。driver_ws/build、install、log 是活动工作区产物，应保留。
后续回滚使用 Git 版本和上述构建命令；不要恢复旧 overlay 路径。

2026-10-09 本次宇泛清理后复验：全新 x86 构建、开发板 ARM64 构建、协议/里程计
及 native DDS 读取测试通过；原 X30/D1 的构建回归通过，相关代码、配置、部署文件
和文档与整理前 Git 版本一致。开发板 setup 环境链已清除旧 overlay，三个旧目录已删除，
删除后新终端的默认 launch 与实际键盘 i/J/l/k 的 Twist 通信验证通过，全程未取权。
历史资料和本次构建/测试记录归档在本地主工作区的 `nav_bridge_cleanup_20261009`，
不纳入源码或运行依赖。本次只整理文件，未创建 Git 提交。
