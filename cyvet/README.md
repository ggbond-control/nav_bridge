# UniUbi Cyvet 适配

代码、构建开关、节点、参数和网络连接统一使用 `cyvet`。当前进度见
[STATUS.md](STATUS.md)。网络第一阶段的
实测依据见 [NETWORK.md](NETWORK.md)，后续证据见 [validation](validation/)。

## 当前交付范围

采用 `RobotBackend → CyvetBackend → uniubi_motion_client → robotServer`。
X30/D1 Max 源码保持原实现；`NAV_BRIDGE_BUILD_CYVET` 默认 OFF。开启时需先
构建仓库内固定版本依赖，关闭时无需宇泛消息包或运动客户端。
官方来源、commit 和保留的许可证见
[versions.json](../third_party/uniubi/versions.json) 与
[PATCHES.md](../third_party/uniubi/PATCHES.md)。首版不链接预编译运动 SDK。

节点启动连接、查询能力和布局、开启观测，但不申请运动控制权。
显式 `stand/ready` 才取权并启动三轴零速 walking；动作服务须读到相应
生效状态才成功。只有导航就绪且持权时接收速度。RPC 与私有 executor
均由一个线程串行处理，停止优先，速度仅缓存最新一条。

断线、抢权、租约失效、急停清空速度并封锁输入。重连只恢复观测；
恢复运动须显式调用 `stand/ready`。输入超时调用 `stopAction` 并确认零速
walking。释放和正常退出先确认停止，再释放；失败不会伪报停止成功。
断网无法保证 RPC 送达，固件失权后的实际制动时间必须实测。

## 构建与启动

Ubuntu 24.04 / ROS 2 Jazzy，已验证 x86_64 与 RK3588 ARM64。
除原项目 ROS 依赖外，Cyvet 需要 `rmw_cyclonedds_cpp`、Cyclone DDS 开发库
与 IDL 工具（Ubuntu 包 `cyclonedds-dev`、`cyclonedds-tools`）。

从新的终端执行，避免混入旧型号或旧临时安装目录：

```bash
# 在同一个 driver_ws 内先构建 uniubi 消息和源码客户端，再构建 Cyvet 节点。
# 确认 driver_ws/src/nav_bridge/config/nav_bridge.yaml 为 robot_type: cyvet。
bash /home/cat/Workspace/driver_ws/src/nav_bridge/cyvet/build.sh \
  /home/cat/Workspace/driver_ws
source /opt/ros/jazzy/setup.bash
source /home/cat/Workspace/driver_ws/install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py
```

本地构建可将脚本路径和工作区改为本地绝对路径；设置
`CYVET_BUILD_TESTING=ON` 会编译并注册 Cyvet 测试。
`third_party/uniubi/COLCON_IGNORE` 防止常规 colcon 扫描引入可选依赖；脚本
通过明确的 `--base-paths` 构建。宇泛依赖因此未列为无条件 package.xml 依赖。
脚本默认工作区是 nav_bridge 所在 src 的上级目录，不需要额外工作区。
依赖首次构建后，正常 source driver_ws/install 即可编译 nav_bridge；
开启 Cyvet 的 CMake 开关仍须明确设置，其他型号关闭时不引入宇泛依赖。
launch 的默认型号直接读取安装后的 config/nav_bridge.yaml，安装不再自动改写型号。

launch 支持 `params_file:=/absolute/cyvet_params.yaml` 和
`cyclonedds_config:=/absolute/cyvet_cyclonedds.xml`。业务沿用调用者的
`ROS_DOMAIN_ID`（早期现场验证用 63，当前开发板终端为 81）；机器人私有 context 固定默认 Domain 42。
未显式指定 cyclonedds_config 时继承环境中的 CYCLONEDDS_URI；该变量也未设置时
才使用包内默认 XML（仅将 Domain 42 绑定 eth1）。继承的配置必须允许域 42 访问机器人。
业务域请使用与机器人域不同的值。配置以开发板 eth1 为准，本地没有此网卡时
请提供实际接口配置，模拟测试自动使用 loopback 和隔离域 174/175。

已确认设备编号 `E03C1CBAD8D2A104`、机器人有线 `169.254.216.210/16`，
开发板 eth1 `169.254.216.1/16`、管理 Wi-Fi `10.0.40.163`。
DDS 服务发现不依赖在参数中写死机器人 IP；通过 device_id 检查 RPC 回复。
原始观测话题没有 device_id，Domain 42 中只能有当前目标机器人的数据源。

## 业务接口

开发板现在已配置默认终端环境，使用标准 ROS 键盘命令，直接使用方法见
[TERMINAL.md](TERMINAL.md)。无需每次设置 DDS 域、XML 或安装工作区。

节点名 `/nav_bridge_node`。接口名称和类型保持现有业务约定。

| 服务（节点私有命名空间） | 类型 | Cyvet 行为 |
|---|---|---|
| `stand`、`ready` | `std_srvs/srv/Trigger` | 显式取权，确认零速 walking，解除本地禁止运动 |
| `lie` | 同上 | 确认 laying，然后释放控制权 |
| `soft_estop` | 同上 | 本地立刻封锁速度，请求官方 emergencyStop；仅 ready/stand 恢复 |
| `release_control` | 同上 | stopAction，确认后释放 |
| `set_gait` | `rcl_interfaces/srv/SetParameters` | 一项 `gait`，仅整数 0 或字符串 WALK/walk；不取权或解除急停 |
| `set_speed` | 同上 | 一项整数 `speed` 1/2/3，改变本地速度上限 |
| `set_body_height`、`charge_command` | 同上 | 明确返回不支持 |

`/cmd_vel` 为 `geometry_msgs/msg/Twist`，使用 linear.x / linear.y / angular.z，
单位 m/s / m/s / rad/s；拒绝 NaN/Inf。三档上限分别为
`(0.2,0.1,0.3)`、`(0.4,0.2,0.5)`、`(0.6,0.3,0.8)`，默认第一档。
实际取配置上限与设备能力范围的交集，不切换厂商 fast profile。
SDK 默认 30 Hz 派发（现场配置为 10 Hz）、500 ms 输入超时、200 ms SDK 默认速度/停止 RPC 超时（现场配置为 500 ms）；
连接 5 s、重连 2 s、动作确认 10 s、数据有效期 2 s、请求租约 5 s。
SDK 取权含厂商约定的 3 s master 切换等待；停止请求会先禁止本地速度，
远程停止须等当前 SDK 调用结束。SDK releaseControl 的 RPC 超时为 5 s。

`lateral_sign` 默认 +1（ROS bridge/mock 的正左约定），可设为 -1；
该项改变横向指令和查询到的横向控制速度。当前设备已现场确认 +1 为正左。

| 话题 | 类型 | 有效性规则 |
|---|---|---|
| `/battery/level` | `std_msgs/msg/UInt8` | 0–100；未知、过期、未连接时不发布 |
| `/robot_basic_state` | `std_msgs/msg/Int32` | 使用下表 Cyvet 通用状态值 |
| `/robot_gait_state` | `std_msgs/msg/Int32` | 0 = WALK（支持模式，连接与就绪看诊断） |
| `/imu/data` | `sensor_msgs/msg/Imu` | 有限值且 accel/gyro 无错误；四元数归一化，无效 orientation covariance[0]=-1 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 实际布局名称；所有关节在线、无错误、数据完整且名称唯一才发布 |
| `/leg_odom` | `nav_msgs/msg/Odometry` | 有效且有限的 SE(2) 样本；epoch/重连以偏移连续衔接 |
| `/robot_fault` | `std_msgs/msg/String` | 连接、控制及 RPC 故障文本 |
| `/nav_bridge_node/backend_status` | `std_msgs/msg/String` | JSON：连接、持权、导航就绪、电量有效性、动作、三轴控制速度、缓存和最近错误 |

基本状态：0 UNKNOWN、1 LYING_DOWN、2 STANDING_UP、3 STANDING、4 MOVING、
5 LOCKED、6 ESTOP。当前固件动作映射实际发布 0/1/3/4/6；walking 零速映射 3，
非零速映射 4。它们是通用桥接状态，**不能按 X30 固件枚举解释**。
诊断中的速度是运控查询结果，并非独立测得的实际机身速度。

默认 frame 为 odom / base_link / imu_link；不发布 odom→base_link TF。
停止 RPC 失败会封锁输入并请求 emergencyStop，随后持续重试/确认停止；
确认前拒绝导航准备。恢复仍需显式 stand/ready。诊断 stop_pending 表示
停止待确认；失去控制权后本机无法再保证远程停止。

速度 RPC 的 deadline 超时允许一次有限恢复：查询到 fresh walking、仍持权且
未被停止取消，只重试期间收到的更新、未过期速度。明确拒绝、错误设备、过期输入
或再次失败仍封锁并停止，不会自动 stand/ready。诊断提供 velocity_rpc_timeouts
和 velocity_rpc_recoveries，终端打印恢复 WARN 或具体 fault。
本固件 DDS 可能选择机器人 Wi-Fi locator，板端已增加目标专用有线路由；
实测原因、路由维护和回滚见 [RPC_TIMEOUT_FIX.md](RPC_TIMEOUT_FIX.md)。

消息使用业务节点接收时间，原始观测时间仅用于拒绝重复和倒退样本。
当前协议时间基准未完成实机标定，不能据此声称测量时间已与 ROS 对齐。
数据缺失不会制造关节或里程计；重连缺失期间不推算位移。

## 验证与导航接入

```bash
source /opt/ros/jazzy/setup.bash
source /home/cat/Workspace/driver_ws/install/setup.bash
export ROS_DOMAIN_ID=63 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/cat/Workspace/driver_ws/src/nav_bridge/config/cyvet_cyclonedds.xml
python3 /home/cat/Workspace/driver_ws/src/nav_bridge/cyvet/read_only_smoke.py \
  --launch --params /home/cat/Workspace/driver_ws/src/nav_bridge/config/cyvet_params.yaml
```

运动脚本 **不会自动执行**，需现场可接管且区域清空；先另一个终端启动节点。
逐项执行 `python3 cyvet/motion_smoke.py --stage stand|axes|estop|lie`
（每次选一个）；--stage zero_stream 提供零速连续通信诊断。axes 每轴正负方向各 1 s，速度仅 0.1 / 0.06 m/s、
0.1 rad/s，停止输入后等待 watchdog 确认，最后释放。
可用 `--seconds 5` 延长观察；使用 `--executable /path/to/cyvet_nav_bridge_node
--params /path/to/cyvet_params.yaml` 可由脚本启动、清理独立节点。软件状态成功不能代替
现场方向、姿态和停止距离观察；脚本失败会尝试停止与释放。低速可能无法明显观察，当前设备已验证
`--axes lateral_yaw --seconds 5 --lateral-speed 0.1 --yaw-speed 0.3`。

`--stage combined --seconds 3 --lateral-speed 0.1 --yaw-speed 0.3` 依次发送
前进/左移/左转和后退/右移/右转组合，各项仍在第一档上限内；两组之间确认停止。
`--stage takeover` 仅零速 walking，提示现场在 60 秒内遥控接管；检测失权后
继续观察 5 秒，确认零速输入不会自动取权，不再调用 ready。此项尚未现场执行。

复验现有导航就绪脚本（会显式站立，需现场配合）可执行：

```bash
python3 /home/cat/Workspace/driver_ws/src/nav_bridge/cyvet/readiness_smoke.py \
  --readiness-script /home/cat/Workspace/task_ws/src/inspection_bringup/scripts/wait_for_ready.py \
  --params /home/cat/Workspace/driver_ws/src/nav_bridge/config/cyvet_params.yaml
```

该脚本使用安装中的默认型号选择，电量就绪后调用 stand，最后 lie/release 并清理进程。

本地自动测试：

```bash
colcon test --base-paths /absolute/nav_bridge --packages-select nav_bridge \
  --build-base /absolute/cyvet_workspace/build --install-base /absolute/cyvet_workspace/install \
  --ctest-args -R cyvet_ --output-on-failure
colcon test-result --test-result-base /absolute/cyvet_workspace/build/nav_bridge/test_results
```

集成测试是 ROS 协议模拟服务，不是官方二进制 mock，也不是真机；覆盖只读启动、
错误设备身份、取权/动作拒绝、无效数据、速度限幅、输入超时、RPC 延迟、
运动状态过期、急停恢复、并发停止、人工抢权、重连不重放、epoch 连续和正常退出。
模拟断线通过 RPC 错误响应实现，不能替代真实拔线及固件租约验收。

现有 `inspection_bringup` 已按 battery/level 就绪后调用 stand，可直接复用
服务类型。其 navigation.launch.py 未传 robot_type；开发板源码的
config/nav_bridge.yaml 已明确选择 cyvet，安装复制相同配置。多型号构建可修改源码选择文件后安装，
或提前独立启动 Cyvet 并在 bringup 禁用重复启动 nav_bridge。巡检路线步态只能 WALK/0，关闭 system.yaml
的 modules.charge 及 routes.yaml 的 route.charge_config.enabled；不得复用 X30 步态编号。
真机已取得 Walk 里程计和 12 关节数据；方向、遥控抢权、退出/拔线停止、
短路线和 30 分钟运行的最终验收进度见 STATUS.md，完成前不配置自动启动。

## 部署与回滚

本地源码是修改源。开发板源码同步前检查 Git 差异，并将完整原目录（含 Git）
备份到 `/home/cat/Workspace/backups/`。Cyvet 与宇泛源码依赖均在 driver_ws 内构建安装，
早期 cyvet_adapter_ws 是联调隔离目录，当前不再加载。迁移前 driver_ws 的旧 nav_bridge
build/install、.zshrc 和源码型号选择备份位于
`/home/cat/Workspace/backups/cyvet_driver_ws_migration_20261009/`。
回滚先停止 Cyvet 进程，保留当前 build/install/nav_bridge，恢复备份中的同名目录，
恢复需要的源码与型号配置，打开新终端 source driver_ws/install。
需恢复早期 Cyvet 隔离部署可恢复该迁移备份里的 zshrc_before（仍指向 cyvet_adapter_ws）。
网络回滚见 NETWORK.md。
正常停止使用 SIGINT/SIGTERM；SIGKILL 无法执行停止与释放清理。
