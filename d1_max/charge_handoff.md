# D1 MAX 充电退出与导航交接

`charging_pile_connected` 表示充电桩蓝牙连接，不代表机器人在桩上。
电池 `CHARGING` 才表示正在充电；充电任务运行中、实际充电中、脱桩任务运行中分别上报为 `/charge_manager_state` 的 1、2、3。

`/nav_bridge_node/charge_command` 的 `4 / UNDOCK_START` 保持统一的“确认退出完成”语义：

- 新鲜状态确认蓝牙未连接，且没有活动 SDK 充电/脱桩任务时，直接成功。
- 蓝牙连接但电池未充电、没有活动任务及待完成的脱桩需求时，直接成功。
- 充电任务运行中时，先停止并等待 SDK 确认；若蓝牙仍连接，保留脱桩需求，不能因为 STOP 后电池不再充电就跳过脱桩。
- 已运行的 SDK 脱桩任务必须等待 `UNDOCK SUCCESS`；再收到完成后的新鲜状态，确认电池未充电或蓝牙已断开，才能成功。
- 满电或 STOP 不清除已有脱桩需求；完成交接后清除。状态缺失、过期、SDK 任务失败或超时不放行导航。

蓝牙连接且电池状态未知时，不把默认值当作未充电，也不盲目发送脱桩命令。机器人 SDK 连接失败不等于充电桩蓝牙未连接。

退出确认成功后，`/charge_manager_state` 使用相同判据上报 `0 / idle`，包括蓝牙已断开但电池仍保留 `CHARGING` 的情况，避免服务成功而执行器一直等不到 idle。新的充电/脱桩 SDK 活动，或蓝牙重新连接且电池充电/未知，会使退出确认失效；过期遥测也不能复用该确认。电池原始信息不做改写。

修改只在 D1 backend 内，上层服务、调用顺序、步态实现及 X30 backend 不变。
日志中的 `pile_bluetooth_connected` 明确表示蓝牙连接。

充电命令（含 `3 / QUERY`）串行执行。若 START 的响应在传输层丢失，上层可等待一个成功的 QUERY 响应确认前一服务调用已收尾，然后发送 `4 / UNDOCK_START`；QUERY 成功本身不表示脱桩完成。X30 的充电服务也使用相同的串行约束。

D1 bringup 的充电服务等待为 180 秒，覆盖蓝牙连接 90 秒、充电确认 60 秒及 SDK 调用余量。D1 systemd 配置通过 `ZENOH_CONFIG_OVERRIDE=queries_default_timeout=600000` 将会话查询超时设为 600 秒，覆盖独立交接及取消收尾；现有备份的 session/router 同样调整为 600 秒。更新 `config/models/d1m/services.yaml` 后，需用 bringup 服务管理脚本重新安装服务包装脚本使环境变量生效；仅重新编译节点不足以更新已安装包装脚本。

本地验证：

```bash
cmake --build /home/jazzy/drive_ws/build/nav_bridge --target d1_max_nav_bridge_node d1_max_undock_test --parallel 2
ctest --test-dir /home/jazzy/drive_ws/build/nav_bridge -R '^d1_max_undock_test$' --output-on-failure
```

测试使用 SDK 命令替身，执行真实 backend 的入口、状态回调和等待逻辑，不连接真实机器人。
