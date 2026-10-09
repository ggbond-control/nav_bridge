# Cyvet 厂商模型切换

沿用原有 RobotBackend、Cyvet 独立型号节点、统一 launch 和 SetParameters 类型的
`/nav_bridge_node/set_gait`。X30 将 gait 映射为固件步态；D1 Max 的通用模式中
WALK/0 映射慢速、RUN/3 映射高速。Cyvet 复用这两个名称与编号，发送厂商
walking 参数 `controlProfile`，不修改 X30 或 D1 的实现。

官方文档当前只提供字符串 `slow` / `fast`，没有独立 `medium`。
来源：[High-level control](https://github.com/uniubi-ai/uniubi-docs/blob/main/docs/how-to/high-level-control.zh-CN.md#选择-walking-控制档位)，
固定文档 commit `939d9c7286c35089c3464a2fbd04e67914b510a1`；2026-10-09
重新下载 main 文档仍为两档。本设备能力响应没有 profile 列表，不能从三个本地
速度上限推断有三个厂商模型，也不能从公开资料核实模型权重或内部实现。

| set_gait 输入 | 厂商参数 | /robot_gait_state |
|---|---|---|
| 整数 0，字符串 WALK / SLOW | controlProfile="slow" | 0 |
| 整数 3，字符串 RUN / FAST | controlProfile="fast" | 3 |

字符串不区分大小写；MEDIUM、SILENT/34 及其他步态明确拒绝。
兼容 `set_speed` 服务保留：speed=1 选择 slow，speed=3 选择 fast；speed=2 拒绝。
这不是宇泛原生的三档 API。建议业务统一使用 set_gait。

## 使用

```bash
ros2 launch nav_bridge nav_bridge.launch.py
```

另一个终端显式站立，每次 stand/ready 都选配置中的默认模型（交付默认 slow）：

```bash
ros2 service call /nav_bridge_node/stand std_srvs/srv/Trigger '{}'
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

切换 fast：

```bash
ros2 service call /nav_bridge_node/set_gait rcl_interfaces/srv/SetParameters \
  '{parameters: [{name: gait, value: {type: 4, string_value: FAST}}]}'
```

切回 slow：

```bash
ros2 service call /nav_bridge_node/set_gait rcl_interfaces/srv/SetParameters \
  '{parameters: [{name: gait, value: {type: 2, integer_value: 0}}]}'
```

切档先封锁输入、清空缓存，再 stopAction/确认停止，然后发送三轴全零的新模型参数。
仅确认成功后恢复输入；旧 cmd_vel 不重放。失败或停止抢占不会自动恢复，需显式
stand/ready。启动、重连仍只读，set_gait 不自动取权或解除急停。

删除了 speed_limits_slow/medium/high 和适配层裁剪逻辑。默认键盘输入 0.5 m/s、
1.0 rad/s 现在完整发送，速度由键盘和厂商模型控制。输入仍拒绝 NaN/Inf，
500 ms watchdog、RPC 失败停止、控制权管理和停止优先保持有效。
官方定义的 slow 范围为 vx [-1.5,1.5]、vy [-1,1]、yaw [-2.5,2.5]；
fast 为 vx [-2.5,3]、vy [-1,1]、yaw [-3,3]，单位 m/s、m/s、rad/s。
这不是实测最大速度，实际可用范围和越界处理取决于当前固件。

## 状态确认边界

`/nav_bridge_node/backend_status` 增加 control_profile_selected、
control_profile_acknowledged、control_profile_reported 和 gait。
selected/acknowledged 为最近成功准备或切换的模型，reported 为最近状态回复中的
controlProfile；固件不报告时为空字符串。失权后这些历史选择不能代表设备当前模型，
必须同时查看 connected/control_owned/navigation_ready。

服务成功要求厂商 RPC 接受及 walking/三轴零速状态确认；固件若报告 profile，
还检查与目标相同。固件不报告 profile 时会在服务结果中明确说明，不能将 RPC
接受和 walking 状态冒充模型权重已验证。厂商明确拒绝/身份错误/超时不会静默
回退 slow 后报告 fast 成功。

本设备固件 Cyvet-V1.00.000 在显式传 profile 后会回报名称。2026-10-09
零速实机已确认 slow→fast→slow→fast，并确认 ready 恢复 slow、lie/释放成功。
RPC 回复早于实际档位状态更新，确认时会持续轮询目标档位；超时不匹配仍失败。
实机记录 [profile_physical_zero_20261009.txt](validation/profile_physical_zero_20261009.txt)；
首次立即判断导致的失败记录保留。未测试 fast 的非零运动表现或最高速度。

`motion_smoke.py --stage profiles` 是现场协调用的零速检查：slow 启动→fast→slow→fast
→ready 恢复默认 slow→lie/释放。不会发布平移或转向速度，不会自动执行。
