# X30 自主充电

`nav_bridge_node` 提供 `~/charge_command` 服务，用于向 X30 感知主机发送自主充电指令。

## 服务

```bash
ros2 service call /nav_bridge_node/charge_command rcl_interfaces/srv/SetParameters \
"{parameters: [{name: 'charge_command', value: {type: 2, integer_value: 0}}]}"
```

`charge_command` 支持整数或字符串。整数取值：

| 值  | 名称  | 底层 UDP             |
| --- | ----- | -------------------- |
| `0` | START | `0x91910250 value=1` |
| `1` | STOP  | `0x91910250 value=0` |
| `2` | RESET | `0x91910250 value=2` |
| `3` | QUERY | `0x91910253 value=0` |

字符串取值支持 `start`、`stop`、`reset`、`query`，大小写不敏感。

START 前会向 `192.168.1.105:43899` 发送 `0x3101EE03 value=2`，将速度源切到导航模式；充电管理指令发送到 `192.168.1.105:3333`。

如果机器人处于 RL 模式，START 会先调用现有站立动作退出 RL，再发送充电指令。

## 返回

响应使用 `rcl_interfaces/msg/SetParametersResult`：

| 字段           | 说明                             |
| -------------- | -------------------------------- |
| `successful`   | 是否收到符合该命令预期的充电状态 |
| `reason`       | JSON 文本，包含 `charge_state`、`state_name`、`message` |

常见状态：

| 状态码   | 名称                  | 含义             |
| -------- | --------------------- | ---------------- |
| `0x0000` | `idle`                | 空闲             |
| `0x0001` | `do_charge_task`      | 正在执行充电任务 |
| `0x0002` | `charging`            | 正在充电         |
| `0x0003` | `do_over_charge_task` | 正在结束充电任务 |
| `0x0004` | `pile_error`          | 充电桩断电       |
| `0x0005` | `safety_warning`      | 请先拔掉充电器   |

## 等待行为

START 不使用固定超时判断成功失败。服务会持续等待充电管理器进入 `do_charge_task` 或 `charging`，如果收到明确错误状态则返回失败。

STOP 会持续等待充电管理器从 `do_over_charge_task` 回到 `idle`，成功后自动切回手动模式。RESET、QUERY 会等待充电管理器返回对应状态。

## 参数

充电相关网络参数位于 `config/x30_params.yaml`：

| 参数                 | 默认值          | 说明                    |
| -------------------- | --------------- | ----------------------- |
| `charge_host_ip`     | `192.168.1.105` | 感知主机 IP             |
| `charge_host_port`   | `3333`          | 自主充电请求端口        |
| `charge_config_port` | `43899`         | 速度源/导航模式配置端口 |
| `charge_local_port`  | `49004`         | 本地源端口              |


## X30 有界充电服务

X30 的 `4 / UNDOCK_START` 使用 STOP → 新响应 idle → 手动模式确认，保留与 D1_MAX 相同的“确认退出完成”语义，但不调用 D1 的物理脱桩 SDK。
`charge_command_timeout_ms` 默认 60000，覆盖排队、START 前姿态准备、充电状态等待及 STOP 后模式确认。没有 UDP 响应、状态始终不匹配或未知状态（如现场的 `0x100B`）均在截止时间内返回失败，错误包含最后收到的状态；未知状态不会被解释为脱桩成功。
充电服务由独立工作线程串行执行，最多排队 4 个请求；ROS 控制与状态回调不再被充电等待占住。排队超时的动作不会补发。成功的 QUERY 仍只证明前一服务调用已经结束，不表示物理退出已完成。

本地隔离验证：

```bash
ctest --test-dir ~/drive_ws/build/nav_bridge -R '^x30_(action_deadline_test|charge_timeout_integration)$' --output-on-failure
```
