# X30 自主充电

`nav_bridge_node` 提供 `~/charge_command` 服务，用于向 X30 感知主机发送自主充电指令。

## 服务

```bash
ros2 service call /nav_bridge_node/charge_command nav_bridge/srv/ChargeCommand "{command: 0}"
```

`command` 取值：

| 值  | 名称  | 底层 UDP             |
| --- | ----- | -------------------- |
| `0` | START | `0x91910250 value=1` |
| `1` | STOP  | `0x91910250 value=0` |
| `2` | RESET | `0x91910250 value=2` |
| `3` | QUERY | `0x91910253 value=0` |

START 前会向 `192.168.1.105:43899` 发送 `0x3101EE03 value=2`，将速度源切到导航模式；充电管理指令发送到 `192.168.1.105:3333`。

如果机器人处于 RL 模式，START 会先调用现有站立动作退出 RL，再发送充电指令。

## 返回

响应字段：

| 字段           | 说明                             |
| -------------- | -------------------------------- |
| `success`      | 是否收到符合该命令预期的充电状态 |
| `charge_state` | 充电管理器状态码                 |
| `state_name`   | 状态码文本                       |
| `message`      | 结果说明                         |

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
