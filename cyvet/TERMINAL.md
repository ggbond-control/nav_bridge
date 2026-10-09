# 开发板默认启动与键盘控制

2026-10-09 已修改 `/home/cat/.zshrc` 和
`/home/cat/Config/cyclonedds_large_message.xml`，并重新安装 nav_bridge。
修改前的两个文件备份在
`/home/cat/Workspace/backups/cyvet_terminal_20261009_115421/`。

## 配置与工作区

源码是 `/home/cat/Workspace/driver_ws/src/nav_bridge`，编译安装已迁入同一个
`/home/cat/Workspace/driver_ws`。新终端使用原有 driver_ws/algor_ws/task_ws 环境，
不再加载额外的 cyvet_adapter_ws overlay。
`ros2 pkg prefix nav_bridge` 应返回
`/home/cat/Workspace/driver_ws/install/nav_bridge`。

launch 读取上述安装包里的 `config/nav_bridge.yaml`，其中为 `robot_type: cyvet`，
与开发板源码配置一致。已移除构建时自动改写安装型号的逻辑。
本地通用源码默认仍为 x30，开发板的 cyvet 选择是已记录的设备部署配置差异。

`.zshrc` 保持业务域 `ROS_DOMAIN_ID=81` 和 Cyclone RMW，仍使用原 XML 路径。
XML 中原有环回接口、单播发现、大消息和缓冲设置限定于业务域 81；
独立域 42 使用 eth1 和多播发现。业务节点、键盘都在开发板运行；
该环回配置不允许其他电脑直接通过域 81 控制，远程操作应 SSH 到开发板。
launch 优先继承终端 XML；显式 cyclonedds_config 参数仍可覆盖。

2026-10-09 排查发现机器人 DDS 还会广播 Wi-Fi locator，绑定 eth1 仍可能通过
wlan0 发送单播控制请求。已在 cyvet-link 中持久化目标路由
`10.0.40.244/32 via 169.254.216.210 dev eth1`；机器人 IP 改动时必须相应更新。
诊断/回滚方法见 RPC_TIMEOUT_FIX.md。仅靠 XML 的接口配置不能保证这台设备
全部 RPC 经有线发送。

## 使用

打开两个开发板终端；已有终端先执行 `source ~/.zshrc`。

终端 1：

```zsh
ros2 launch nav_bridge nav_bridge.launch.py
```

启动保持只读。等日志提示 Cyvet connected 后，在终端 2 显式站立取权：

```zsh
ros2 service call /nav_bridge_node/stand std_srvs/srv/Trigger '{}'
# 等待 success=True，再运行键盘。
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

标准键盘节点发布 Twist 到 `/cmd_vel`，其默认线速度为 0.5 m/s、转速为
1.0 rad/s；桥接直接发送这些输入给当前厂商模型，不再裁剪为旧第一档速度。
也可添加 `--ros-args -p speed:=0.1 -p turn:=0.3` 使用较小输入。
此前新增的 cyvet_* 快捷别名已移除，没有新增业务接口。

| 按键 | 动作 |
|---|---|
| i / , | 前进 / 后退 |
| 大写 J / 大写 L（Shift+j/l） | 左移 / 右移 |
| 小写 j / 小写 l | 左转 / 右转 |
| u / o | 前进并左转 / 右转 |
| k 或空格 | 零速度 |
| Ctrl+C | 退出键盘并发送零速度 |

Jazzy 的该键盘版本每次按键发布一条消息，持续运动需要按住按键让终端重复输入。
没有新按键消息超过 500 ms，桥接执行停止；这与退出或释放控制权不同。
键盘自身 q/z 等键能修改输入速度。厂商 slow/fast 模型通过原 set_gait 服务切换，
见 [CONTROL_PROFILES.md](CONTROL_PROFILES.md)；每次 stand/ready 默认回到 slow。

退出键盘后，在终端 2 执行：

```zsh
ros2 service call /nav_bridge_node/lie std_srvs/srv/Trigger '{}'
# 或调用 /nav_bridge_node/release_control：确认停止并释放
```

急停可在第三终端调用 `/nav_bridge_node/soft_estop`（std_srvs/srv/Trigger）；
恢复必须再次显式调用 stand。
退出桥接使用 Ctrl+C，触发正常停止和释放清理。

## 验证与回滚

keyboard_readonly_smoke.py 会启动新的 zsh 终端和默认 launch，通过伪终端
向实际键盘节点发送 i/J/l/k，检查 Twist 和真实连接状态；不调用 stand/ready。
该项验证软件与 DDS 连通性，运动方向已由之前的六方向实机测试确认。
检验必须在没有运行桥接或其他速度发布器时执行。

回滚终端配置：先停止节点，将上述备份目录中的 `.zshrc` 和
`cyclonedds_large_message.xml` 分别复制回原路径，打开新终端。
driver_ws 迁移前的 nav_bridge 安装产物已备份，构建回滚见 README.md。
迁移备份中的 zshrc_before 会恢复早期隔离工作区加载；当前无需加载该工作区。

## 最终交付目录清理

`backups` 是回滚资料，没有运行依赖，验收后归档即可删除。
`uniubi_network_ws` 是第一阶段探测工作区，当前运行不再加载它；关键证据已保存
到 nav_bridge/cyvet/validation，可归档或删除。NETWORK.md 中旧命令是历史记录。
`cyvet_adapter_ws` 是早期联调工作区，已迁移至 driver_ws，当前不再加载。
迁移验收通过后可归档删除；本次保留该目录以支持回滚。
