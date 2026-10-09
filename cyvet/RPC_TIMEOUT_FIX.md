# 2026-10-09 键盘控制间歇超时排查

用户在 stand 后键盘控制一段时间，出现 setMotionActionParams RPC 超时，
桥接封锁输入；还有 renewMotionControl 超时及现场自动趴下。
继续发 cmd_vel 不会恢复导航就绪，必须显式 stand/ready。

## 已证实的网络问题

机器人 DDS participant 广播了有线和 Wi-Fi 地址。虽然客户端域 42 绑定 eth1，
Cyclone 仍选择远端 Wi-Fi locator 10.0.40.244；Linux 的默认路由把这类请求从
wlan0 发出，源地址仍是 169.254.216.1。回复则从机器人有线地址返回。
因此“绑定 eth1”并不能保证控制链路全程使用有线。

只读复现，不取权：10 Hz 查询 queryMotionState/getSystemStatus，单请求窗口
60 秒内 349 次请求有 13 次等 2 秒未回复；窗口 3 时 531 次有 34 次未回复。
抓包证明请求目标为 10.0.40.244，经 Wi-Fi 路由；并非仅由键盘输入造成。

## 部署修复

在 NetworkManager cyvet-link 中添加一条目标专用路由：

```bash
sudo nmcli connection modify cyvet-link +ipv4.routes '10.0.40.244/32 169.254.216.210 50'
sudo nmcli device reapply eth1
ip route get 10.0.40.244 from 169.254.216.1
```

命令已在板端执行，不应重复添加。路由结果须包含
`via 169.254.216.210 dev eth1`。抓包确认请求的目标 MAC 为机器人有线
e0:3c:1c:ba:d8:d2。这是让机器人广播的 Wi-Fi locator 也通过同一设备有线入口
到达 robotServer；管理 Wi-Fi 默认路由保持不变。
该路由依赖当前确认的两个机器人 IP，若 App 改动 IP，部署路由须相应更新。
仅在本设备验证，不代表其他固件/设备允许相同跨接口 IP 接收。

尝试 prefer_multicast=true 时 RPC 全部失败（45/45），没有采用该配置。
实际 DDS XML 和 bridge 的 10 Hz/500 ms/5 秒租约均保持原值。
不会用延长超时、自动重新站立或忽略失权来掩盖通信故障。

修复后首轮 594 次查询无超时，延迟中位数 5.8 ms、p99 25.9 ms、最大 32.7 ms。
增加独立查询客户端与零速控制负载的首次复验仍有一次速度超时、两次查询缺回复，
失败记录保留；随后独立 60 秒零速持续控制通过。继续压力验证结果见 validation。

并发查询/零速负载的抓包进一步证明：少量请求已经发到设备，设备约 5.006 秒后
返回 `{"code":1,"result":false}`，而相邻请求通常 3–30 ms 成功。
这属于机器人内部请求转发的间歇失败；客户端路由修复不能宣称修复了固件根因。

## 速度超时的有限恢复

SDK 将真实 deadline expiry 单独报告为 kRpcTimeout。只有此种失败，桥接才查询
有效运动状态，然后检查仍持权、仍导航就绪、动作 walking、代次未被停止取消，
并且收到比失败命令更新、仍在 500 ms 内的输入，才发送一次最新速度。
不重放失败命令；明确拒绝、错误设备、输入过期、停止/抢权和再次失败仍封锁并停止。
没有自动取权、站立或解除急停。收到更新输入并不能授权恢复被封锁的导航状态。
diagnostics 新增 velocity_rpc_timeouts / velocity_rpc_recoveries。
恢复成功会输出 WARN，未能恢复会输出具体 fault。

新版真机两分钟零速控制、同时 10 Hz 独立只读查询加压通过。出现一次速度超时，
一次有限恢复成功，导航就绪未丢失，最后停止确认并释放；约 6000 条 IMU/关节样本。
独立查询客户端仍有 5/1187 次 2 秒未回复，记录保留，不能宣称固件间歇失败已消除。

## 空闲续租修复

90 秒不发 cmd_vel 的测试再次触发续租超时和失权。旧 SDK 在 lease/3 后发送续租，
再等待固定 3 秒回复；5 秒租约留给重试的时间过短。
修改为每次等待不超过 lease/10（最少 200 ms、最多 3 秒）且不越过当前到期时间，
5 秒租约单次等待为 500 ms；未回复可提前重试。没有有效响应或控制动作不会
刷新本地租约，真正失权/到期仍立即封锁。集成测试使用服务端返回的 2 秒短租约
并让首个回复迟到 3 秒，验证提前重试而不重新取权。

路由已确认写入 `/etc/netplan/90-NM-9af00675-e1cc-4bb9-98bc-9b4054e3d78e.yaml`，
由 NetworkManager 的 netplan 后端生成当前连接。未执行机器重启验收。

新版真机 90 秒 idle_hold 通过：两次续租超时警告均在租约到期前重试恢复，
全过程保持控制权和导航就绪，最后确认停止并释放。见 rpc_renew_idle90_20261009.txt。
随后六方向各 3 秒速度反馈、watchdog 停止及释放均通过，无速度超时或故障；
里程计横向 +0.163/-0.256 m、转向 +0.722/-0.775 rad，IMU 正负转速对应。
本轮现场观察反馈单独记录，不将控制状态成功等同物理停止距离验收。
用户最终确认：“没有出现，本次测试一切正常，完全符合预期。”

## 故障可见性与回滚

节点会将实际 backend fault 限频输出到启动终端，并继续发布 /robot_fault 和
/nav_bridge_node/backend_status，避免终端仅剩 Explicit stand/ready required。
停止失败回退、失权封锁、清空缓存及显式恢复契约保留。

网络修改前状态保存于
`/home/cat/Workspace/backups/cyvet_rpc_route_20261009/`。
若需回滚这一条路由：

```bash
sudo nmcli connection modify cyvet-link -ipv4.routes '10.0.40.244/32 169.254.216.210 50'
sudo nmcli device reapply eth1
```

真实拔线后的设备自主停止行为仍须单独验收；本文压力测试不代替拔线保证。
