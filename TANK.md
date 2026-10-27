# TANK HTTP 底盘接入

TANK 是本项目为新增四轮巡检车定义的平台标识。首期接入运动、软件急停、
工作状态、电池和断线恢复；不发布 IMU、里程计、关节或虚构的充电状态。

## 构建与启动

依赖：ROS 2、libcurl、JsonCpp、pkg-config。Ubuntu 可安装
`libcurl4-openssl-dev libjsoncpp-dev pkg-config`。

```bash
colcon build --packages-select nav_bridge --cmake-args -DNAV_BRIDGE_BUILD_TANK=ON
source install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py robot_type:=tank
```

默认地址为需求提供的 `49.235.44.225:10050`。部署前在
`config/tank_params.yaml` 中设置实际地址，也可通过自定义 `params_file` 启动。
直接运行时同样支持 ROS 参数：

```bash
ros2 run nav_bridge tank_nav_bridge_node --ros-args \
  -p host_ip:=192.168.1.20 -p host_port:=10050 -p cmd_vel_rate_hz:=10.0
```

所有参数在启动时应用，设为只读以避免运行中修改参数但后端未应用。
修改 YAML/CLI 后重启生效。原有机器人仍使用各自构建选项与 launch 分支。

## 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `host_ip` / `host_port` | `49.235.44.225` / `10050` | HTTP 主机与端口 |
| `connect_timeout_ms` | 500 | TCP 连接超时，实际不超过请求总超时 |
| `http_timeout_ms` | 3000 | 状态、急停、初始化等请求的总超时 |
| `move_timeout_ms` | 1000 | 单次运动请求总超时，与发送周期分开配置 |
| `reconnect_interval_ms` | 2000 | 失败后的重连间隔 |
| `cmd_vel_rate_hz` | 10.0 | 独立线程发送速度，允许 3..100 Hz |
| `cmd_vel_timeout_ms` | 500 | 上层速度有效时间，超时后发零速度 |
| `max_linear_speed` | 1.2 | m/s，允许降低，不能超过 1.2 |
| `max_angular_speed` | 1.0 | rad/s，暂定值，需实机确认 |
| `state_poll_interval_ms` | 1000 | 状态、急停查询间隔 |
| `battery_poll_interval_ms` | 3000 | 电量查询间隔 |
| `raw_vel_poll_rate_hz` | 10.0 | 底盘原始速度独立查询的目标频率，允许 (0,100] Hz |
| `raw_vel_timeout_ms` | 500 | 原始速度单次查询超时 |
| `telemetry_timeout_ms` | 10000 | 状态反馈失效时间，须大于状态间隔 + 两次 HTTP 超时 |
| `state_publish_rate_hz` | 10.0 | ROS 状态/诊断发布频率，允许 (0,100] |
| `startup_acquire_control` | true | 启动确认 Idle 且急停未开启时开始发送零速度 |
| `enable_motion_output` | true | false 时仅查询，不切换模式、不解除急停、不发送任何速度帧，退出也不发送 |

仅诊断真实设备、禁止任何速度输出时，用 `enable_motion_output:=false`。
此模式下 ready/stand/lie/soft_estop/release_control 都返回控制已禁用。
状态返回的 `state_initialized` 仅记录到诊断，不作为连接成功条件；实机可以在
Idle 状态返回 `successed=true`、`state_initialized=false`。

仓库提供 `config/tank_readonly_params.yaml`，可通过 launch 的 `params_file`
参数指定该文件绝对路径；此配置启动、重连和退出都不会发送控制请求。

协议要求 `/robot/cmd/move` 至少约 3 Hz。实际达到的频率受 HTTP 延迟影响，
配置频率是目标频率。传输超时按下述连续失败/应答有效期判定是否重连；
不积压或突发补发旧指令。
增大请求超时能容忍链路抖动，但在途请求也可能延长停止服务的等待时间。
`~/backend_status` 的 `last_move_interval_ms`、`last_move_request_ms` 和
`move_failures` 可用于检查链路。状态与电池查询独立于速度循环，不承担运动保活。

## 控制与服务

`/cmd_vel` 使用 ROS 标准坐标：前进 x 为正、左转 z 为正。后端将其映射为
`linearSpeed=vx`、`angularSpeed=vyaw`，忽略侧向 vy。
当前 TANK 根据实机方向反馈直接透传角速度符号；所提供文档描述的右转为正
与该实机表现不一致，因此这里不再反转 ROS 的 angular.z。
拒绝 NaN/Inf 并清空上一条速度。订阅队列深度为 1，后端只保存最新指令。

| 服务 | 类型 | 行为 |
|---|---|---|
| `~/ready` / `~/stand` | Trigger | 切换并确认 Idle，解除并查询确认软件急停，发送零速度；随后等待新速度 |
| `~/lie` | Trigger | 本地先禁止运动，再实际发送零速度；必须 ready/stand 后才能继续 |
| `~/soft_estop` | Trigger | 本地先禁止运动，发送 `brake=true` 并查询确认 |
| `~/release_control` | Trigger | 本地禁止运动、清空指令并发送零速度，停止持续运动请求 |
| `~/set_gait` / `~/set_speed` / `~/set_body_height` / `~/charge_command` | SetParameters | 返回不支持及原因 |

```bash
ros2 service call /nav_bridge_node/ready std_srvs/srv/Trigger '{}'
ros2 service call /nav_bridge_node/soft_estop std_srvs/srv/Trigger '{}'
ros2 service call /nav_bridge_node/stand std_srvs/srv/Trigger '{}'
ros2 service call /nav_bridge_node/lie std_srvs/srv/Trigger '{}'
```

`ready/stand` 会自动解除软件急停，这是明确的恢复入口。启动和自动重连只查询
既有急停，不自动解除。急停失败时仍保持本地禁止运动，并返回失败；不会假报
设备制动成功。设备软件急停开启时，控制接口无效。

停止服务先撤销本地控制，等待已在途的运动 HTTP 请求完成/超时，再发停止命令。
并发 ready 被停止操作取消时不能重新启用运动。断线会丢弃旧速度，重连后只有
新收到的 `/cmd_vel` 才能产生非零控制。主动停止/释放后不会因收到速度自动接管。

单次运动传输失败会清除缓存速度，后续非零输出必须来自新的 `/cmd_vel`。
传输错误连续达到 `move_failure_limit`（默认 3 次），或距最近运动应答超过
`move_failure_timeout_ms`（默认 2000 ms），才判定运动链路断开。
应答有效期在请求之间检查；已在途的 HTTP 请求仍可能等待至自身超时。
状态传输失败按 `state_failure_limit`（默认 3 次）计数，且始终受
`telemetry_timeout_ms` 的状态有效期限制；HTTP/业务拒绝、无效响应仍立即断连。
ready/停止/急停服务本身不容错重试，任何失败都返回失败。

HTTP 客户端复用连接，运动和遥测使用独立租用的客户端。输出只发送最新缓存值，
不会积压 ROS 输入；慢 HTTP 返回后不会额外等待完整周期，也不补发错过的周期。
`cmd_vel_rate_hz` 是目标频率，仍受响应耗时限制；参数在启动时读取，修改后需重启。
键盘发布频率是输入频率，与底盘 HTTP 输出频率不同。超过 `cmd_vel_timeout_ms`
没有新输入时改发零速度，不应为掩盖键盘抖动而取消这一保护。

`Idle` 是设备工作模块状态，不代表车辆静止；设备没有暴露独占租约，
`control_owned` 仅表示本桥允许发送，不保证其他 HTTP 客户端不会同时控制。
诊断推荐使用同义字段 `bridge_output_enabled`；它不是底盘报告的控制权。
`control_owner_known=false`、`control_owner="unknown"`、
`robot_controlled_by_bridge=null` 明确表示不能确认实际控制归属/指令执行。
`cmd_vel_fresh` 和 `cmd_vel_age_ms` 仅说明本桥是否收到有效期内的速度输入，
也不能证明底盘接受该控制。连接/输出许可变化时日志会明确打印这一限制。
`release_control` 是本桥提供的本地停止服务，不是设备的遥控器交接协议，
不要求遥控器接管前显式调用它。

文档 `/robot/notification` 示例中的 `auto_mode` 区分自动/远程驾驶，
不直接标识遥控器与 HTTP 客户端；当前实车推送未包含此字段。
`/robot/ws/notice` 有速度/急停等反馈，但同样未报告控制源。
底盘状态接口文档标注 DR210 专用，当前设备返回 errorCode=103（未收到数据）。
因此在设备提供控制仲裁/使能状态定义前，不推断真实控制权或自动切换输出。
零速度按底盘减速度停车；需要立即制动时调用软件急停。

## 状态反馈

- `/robot_basic_state`：使用 `BackendMotionState` 数值：0 UNKNOWN、3 STANDING、
  4 MOVING、5 LOCKED、6 ESTOP。STANDING/MOVING 根据已应答的指令分类，非实测运动。
- `/robot_gait_state`：0 表示本桥 Idle 模式，不表示物理步态。
- `/battery/level`：有效的 0..100 电量；读取失败时不发布伪造值。
- `/raw_vel`：`geometry_msgs/msg/Twist`，来自 `/robot/sensor_data/raw_velocity`
  的 `data.device_data`，`linear_vel` → `linear.x`、`linear_y` → `linear.y`、
  `angular_rotate` → `angular.z`，其余分量为 0。保留设备反馈符号，不作控制限幅，
  不由已发送指令生成。线速度按接口定义为 m/s，角速度为 rad/s；方向需随实机验收。
  独立线程在本地禁止输出/只读模式下仍查询。每次有效响应发布一次；缺失字段、
  非数值、非有限值、业务失败或超时均不发布，不重复旧值、不伪造停车。
  查询失败单独诊断，不使控制链路断连。实际频率受 HTTP 耗时限制。
  Twist 无设备时间戳，因此它表示收到的最新接口反馈，不能保证底层采样新鲜度。
  `raw_vel_valid`、`raw_vel_age_ms`（距最近有效应答的时间）、`last_raw_vel_error`
  可用于检查本地接收有效性；不改变原有基于应答指令的运动状态分类。
- `~/backend_status`：JSON，包含连接、本地控制、设备模式、急停是否已知、
  实测发送间隔、HTTP 请求耗时和最近错误。
  `actual_move_rate_hz` 按最近 5 秒周期运动请求的开始时间计算（含失败请求，
  不含服务/重连发送的零速度），`move_rate_samples` 为样本数；不是底盘执行频率。
  同时提供连续运动/状态失败次数、最近运动应答/状态数据年龄，以及
  `last_move_error`、`last_state_error`、`last_battery_error`，各通道成功后清除。
  `last_error` 保留最近报告的历史错误，重连成功时清除。
- `/robot_fault`：HTTP 失败、业务拒绝、无效反馈等故障说明。

HTTP 2xx 且 JSON 的布尔字段 `successed=true` 才认为业务应答成功。
ready 和急停服务额外查询确认设备状态。模式或外部急停变化会禁止运动，
需显式 ready/stand 恢复。电池失败不单独使运动链路离线。
ready/stand 查询到急停已经关闭时不重复发送解除指令；确实开启时才发送解除，
并再次查询确认。非 2xx 应答会保留前 1024 字节响应正文（转义为单行），
以便定位设备 HTTP 500 等错误；不会把服务器错误当作成功。

## 自动验证与实机验收

```bash
colcon test --packages-select nav_bridge --ctest-args -R tank_protocol_integration --output-on-failure
colcon test-result --verbose
```

测试只连接 loopback 模拟 HTTP 设备，并使用独立 ROS domain，覆盖限速、方向、
超时归零、急停恢复、并发停止、慢遥测、断线重连、业务错误、分块 JSON 响应、
可配置频率、参数校验和退出归零。实机仍需确认 API 可用性、最大角速度、
链路频率及制动距离。里程计/IMU 留待控制闭环验收后接入。
