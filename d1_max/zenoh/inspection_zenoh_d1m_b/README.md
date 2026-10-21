# D1M-A Zenoh 部署配置备份

本目录当前保存 **D1M-A** 的配置快照，覆盖前的 D1M-B 备份已归档。
目录名 `inspection_zenoh_d1m_b` 按要求保留；当前配置来源以本说明和 `backup_record.json` 为准。

- 来源：`cat@47.99.202.196:20004`，主机名 `ZN`。
- 采集时间：`2026-10-21T10:28:42.093597+00:00`。
- 本次仅只读获取设备配置；没有修改设备文件或重启服务。
- 覆盖前本地完整备份（不递归复制已有 `originals`）：[`originals/20261021T102843Z_before_d1m_a_overwrite/`](originals/20261021T102843Z_before_d1m_a_overwrite/)。已有历史归档保留。

| 本地文件 | D1M-A 原始路径 |
|---|---|
| `router_config.json5` | `/home/cat/Workspace/zenoh_ws/router_config.json5` |
| `session_config.json5` | `/home/cat/Workspace/zenoh_ws/session_config.json5` |
| `inspection-zenoh.service` | `/etc/systemd/system/inspection-zenoh.service` |
| `inspection-zenoh.service.d/priority.conf` | `/etc/systemd/system/inspection-zenoh.service.d/priority.conf` |

## 当前配置

- 中间件：`rmw_zenoh_cpp`，服务：`inspection-zenoh.service`。
- Router：查询超时 `120000` ms；监听 `tcp/0.0.0.0:7447`；连接 `tcp/192.168.168.100:7447`；multicast 和 gossip 均关闭；adminspace 启用。
- Session：查询超时 `120000` ms；连接 `tcp/127.0.0.1:7447`；共享内存关闭；没有显式配置 scouting 或固定 `eth1`。
- 服务环境：`ROS_DOMAIN_ID=28`；设备业务进程此前只读核查为 `ROS_DOMAIN_ID=24`。备份保留设备原始值。
- 服务优先级：`Nice=-5`、`CPUWeight=600`。

逐文件 SHA-256、来源路径、源文件修改时间及采集时服务状态见 [`backup_record.json`](backup_record.json)。四个部署文件均按设备原始字节保存，并在覆盖前再次核验源文件哈希。

## 历史记录

`reports/20261015_eth1.md` 及 `originals/20261015_before_eth1/` 是此前 D1M-B 的历史材料，不代表当前 D1M-A 配置。原始 D1M-B README 和全部覆盖前文件已存入上述归档目录。
