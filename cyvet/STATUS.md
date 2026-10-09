# Cyvet 开发与验收检查点（2026-10-09）

后续继续从本文件开始，避免重复网络配置、重复开发或误用旧临时安装目录。
本地 `nav_bridge` 是修改源；开发板 `/home/cat/Workspace/driver_ws/src/nav_bridge`
是同步副本；构建/安装已统一使用 `/home/cat/Workspace/driver_ws`。
代码、节点、构建开关、配置、NetworkManager 连接均使用 `cyvet`。

## 分阶段结论

| 阶段 | 已完成 | 尚未验收 |
|---|---|---|
| 1 网络/只读 | eth1 持久化配置、DDS Domain 42、身份匹配、40/40 RPC、30/30 ping | 重启网络持久化复验 |
| 2 依赖/构建 | 固定源码依赖；Jazzy x86_64/ARM64；Cyvet 默认关闭；关闭时 X30+D1 构建通过 | 无软件构建阻塞 |
| 3 控制状态机 | 只读启动；显式取权；停止优先；抢权/断线缓存清理；急停/恢复；确认后释放 | 真机人工抢权、真实失联租约行为 |
| 4 控制接口 | 三轴速度/限速；WALK/0；本地三档；不支持接口明确失败；六方向现场确认正确；组合指令及停止状态通过 | 组合动作现场观察确认、长时间稳定性 |
| 5 传感器 | 真电量、IMU、12 关节、Walk 里程计；epoch 连续算法和异常数据测试；无 TF | 完整外参、绝对精度/漂移验收 |
| 6 部署/导航 | 原目录备份与源码同步；统一 driver_ws 安装；现有电量就绪→stand 脚本真机通过；lie/release | 拔线、短巡检路线、30 分钟导航、自启动 |

## 已验证的部署值

- 设备 `E03C1CBAD8D2A104`，固件/大脑 `Cyvet-V1.00.000`，MCU `V1.0.14`。
- 机器人有线 `169.254.216.210/16`；开发板 eth1 `169.254.216.1/16`，
  Wi-Fi 管理 `10.0.40.163`。NetworkManager 为 `cyvet-link`，不接管默认路由。
- 机器人 Domain 42、robotServer、/robotServer/Event；早期现场测试业务域 63，
  当前默认终端和键盘业务域 81。
- `cmd_vel_rate_hz=10`、`rpc_timeout_ms=500`、`cmd_vel_timeout_ms=500`，
  默认速度第一档 `(0.2,0.1,0.3)`。SDK 的初始默认值仍为 30 Hz / 200 ms。
- `lateral_sign=+1`：正 X 前进、正 Y 左移、正 yaw 左转，现场已确认。
- 开发板源码与安装的 config/nav_bridge.yaml 均明确 cyvet，安装不自动改写型号。
  本地通用源码选择仍为 x30，这是记录在案的板端部署配置差异。
  无后台自启动服务，无运动测试自动注册执行。

## 真实测试结果和修复

本地自动测试为 3 个数学 GTest 用例和 1 个 ROS 协议集成测试，共 4 项通过。
模拟服务运行在隔离 loopback 域 174/175，未启动官方二进制 mock。
集成测试包含错误身份、拒绝取权/动作、陈旧/无效数据和电机数量变化、限速、
输入超时、停止拒绝急停回退、急停恢复、并发停止、抢权、重连不重放和正常退出。

真机先验证启动不取权，读到真实电量和约 50 Hz IMU。趴下时无关节/里程计数据，
进入 walking 后取得 12 个实际关节以及有效 Walk 里程计。

30 Hz / 200 ms 下出现速度和停止 RPC 超时；30 Hz / 500 ms 仍曾发生速度超时。
修复停止失败后的 emergencyStop 回退、持续停止确认与本地封锁。
10 Hz / 500 ms 下，10 秒零速、六方向各 1 秒和各 5 秒测试均通过；
速度发送和 watchdog 停止未再出现超时，但首次 DDS 发现及站立切换的状态查询
仍偶发超时，重连/轮询可恢复。**不能据此宣称抖动根因已消除或长时间验收通过。**

低速横移 0.06 m/s、转向 0.1 rad/s 时现场运动不明显。提高到第一档上限
横移 0.1 m/s、转向 0.3 rad/s，各 5 秒后，现场确认左右移动、双向转向均正确。
里程计横移分别约 +0.30 m / -0.45 m，转角约 +1.18 / -1.23 rad；
IMU 平均 z 转速约 +0.209 / -0.219 rad/s。相邻里程计样本最大位移约 4.6 mm。
这是方向和数据连通性验证，不是独立定位精度测试，也不是死区的精确定量结果。

固件急停状态是 `emergencyStop`，不是 `walking`。已修正状态确认，并真机通过
急停后禁止速度、冷却后显式 ready 恢复、停止及释放。急停已生效时释放不会
为确认停止而重新请求 walking。正常主动释放的提前失权事件不再误报为抢权。
lie 服务已确认 laying 并释放；动作生效状态不等于独立测得的姿态完全收敛。

现有 `/home/cat/Workspace/task_ws/src/inspection_bringup/scripts/wait_for_ready.py`
在真实电量数据就绪后成功调用 stand，然后测试调用 lie/release，统一 launch
默认选择 Cyvet，节点干净退出。未启动雷达、定位、规划或巡检任务。

补充两组三轴组合 `(0.1,0.1,0.3)` 和 `(-0.1,-0.1,-0.3)`，各 3 秒，
控制反馈及输入超时停止均通过，最终释放并退出，无速度/停止 RPC 超时。
里程计转角分别 +0.808/-0.930 rad，IMU 平均 z 转速 +0.214/-0.250 rad/s；
组合动作现场观察尚未收到确认，不能标记为完整物理验收。
最终部署只读复验电量 85%，无控制权、无导航就绪、无待确认停止及最近错误，
launch 干净退出。组合测试后再次只读 RPC 4/4 成功，motionOwner=0，
动作 laying，三轴控制速度均为零；板端没有残留 Cyvet 测试进程。

## 证据与继续任务

所有软件/实机记录在 [validation/](validation/)；失败记录保留，不能删除后
仅展示成功结果。关键记录为 software_integration、physical_axes_10hz、
physical_axes_5s、physical_lateral_yaw_calibration、physical_estop_fixed、
physical_navigation_readiness（均带日期 20261009）。

本地 Git 基线 `32833c0f972b76d13edac2fc28d54abfea7039bc`；开发板同步前
干净基线 `eb1dc7f0ae246dbd6d4a87a2c3ec69d0e6af8027`，后者新增了 uniubi 依赖。
没有覆盖 Git 历史或创建提交。同步前完整备份：
`/home/cat/Workspace/backups/nav_bridge_before_cyvet_20261009_1025.tar.gz`。
回滚、构建和启动说明见 [README.md](README.md)。

早期隔离工作区的源码已归档至
`/home/cat/Workspace/backups/cyvet_initial_source_20261009`，
板端活动源码只有 driver_ws 副本。源码一致性见 SOURCE_MANIFEST.sha256。
追加证据为 physical_combined、arm64_final_readonly、final_handoff_state。
用户要求暂缓遥控抢权测试、先整理交付；未开始遥控抢权或拔线测试。

按后续要求调整默认终端：.zshrc 最后加载 Cyvet overlay，并保证重复 source
也能优先选中 Cyvet；原大消息 XML 将环回配置限定到业务域 81，追加 eth1 域 42。
launch 继承环境 XML，默认启动和真实键盘节点的 i/J/l/k 消息只读验证通过，
全程未取权，进程正常退出。使用与配置备份见 TERMINAL.md；证据见
keyboard_terminal_initial（首次清理超时，保留记录）、keyboard_terminal_final。

按用户最新要求移除 cyvet_* 终端快捷别名，改为标准
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` 和既有 ROS 服务。
无参数键盘默认输入 0.5 m/s、1.0 rad/s，仍受桥接第一档限速。
源码选择 x30 与当前安装选择 cyvet 的区别、三个工作区的清理条件已记录于 TERMINAL.md。
当时动态依赖来自 cyvet_adapter_ws/install/uniubi，不能直接删除该工作区；
随后按用户的统一架构要求进行下述迁移。

## 统一 driver_ws 部署

D1 Max 在 nav_bridge 内链接 third_party/robot_sdk 的架构库，无额外控制工作区。
Cyvet 保留 RobotBackend、独立型号节点、统一 launch、cmd_vel 和原有服务设计；
差异仅是厂商通信：D1 SDK 走 UDP，Cyvet 源码 SDK 走独立域 DDS。
宇泛消息与源码客户端仍固定在 third_party/uniubi，build.sh 默认构建到
nav_bridge 所在 driver_ws 的 build/install，不需要另建工作区。
可选依赖先通过明确 base-paths 构建，Cyvet OFF 时不要求它们。

已移除安装时隐藏改写 selector 的逻辑，板端源码显式 robot_type: cyvet；
.zshrc 移除 cyvet_adapter_ws overlay，仍加载原有 driver_ws/algor_ws/task_ws。
driver_ws 原 nav_bridge build/install、.zshrc 与源码 selector 的迁移前备份在
`/home/cat/Workspace/backups/cyvet_driver_ws_migration_20261009/`。
早期两个额外工作区暂保留作回滚，迁移验证后不再属于运行依赖。

迁移验收：ARM64 在 driver_ws 内构建三个包成功；新终端查询三个包 prefix
均为 driver_ws/install，nav_bridge CMake 依赖路径与动态消息库路径均为 driver_ws。
默认 launch + 无参数标准键盘 i/J/l/k 只读验证成功，连接真实设备且未取权，
退出干净。x86 Cyvet 构建及 Cyvet OFF/X30+D1 ON 构建回归通过。
证据见 driver_ws_build、driver_ws_keyboard（日期 20261009）。
通用源码清单为 SOURCE_MANIFEST.sha256；板端部署清单为
SOURCE_MANIFEST_BOARD.sha256，只有 config/nav_bridge.yaml 的型号选择哈希不同。

下一步需要明确现有雷达连接、雷达/IMU 外参、地图目录和短路线，随后做
导航闭环及 30 分钟验证。路线只允许 WALK/0，关闭 system.yaml modules.charge
和路线 route.charge_config.enabled。真实拔线和固件停止/租约失效须单独测量；
模拟断线与 cmd_vel 超时停止均不能替代这项保证。实机验收完成前不启用自启动。
