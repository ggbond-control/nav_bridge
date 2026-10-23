# D1M-A / D1M-B Zenoh 配置备份

本目录的四个部署文件与 **2026-10-23 迁移完成后的 A/B 配置快照一致**，可作为两台 cat 主机的共同配置。目录名 `inspection_zenoh_d1m_b` 保留，不能据此判断文件只适用于 B。本次本地同步未连接或修改机器人。

| 机器 | cat 登录方式 | NX 登录方式 |
|---|---|---|
| D1M-A | `ssh -p 20004 cat@47.99.202.196` | `ssh -J cat@47.99.202.196:20004 robot@192.168.168.100` |
| D1M-B | `ssh cat@10.0.40.237` | `ssh -J cat@10.0.40.237 robot@192.168.168.100` |

两台机器各自使用 `.111`（cat）和 `.100`（NX）。每台 cat 连接自己的 NX；相同内网地址不表示两台机器人共用一块 NX。

| 本地文件 | cat 主机部署路径 |
|---|---|
| `router_config.json5` | `/home/cat/Workspace/zenoh_ws/router_config.json5` |
| `session_config.json5` | `/home/cat/Workspace/zenoh_ws/session_config.json5` |
| `inspection-zenoh.service` | `/etc/systemd/system/inspection-zenoh.service` |
| `inspection-zenoh.service.d/priority.conf` | `/etc/systemd/system/inspection-zenoh.service.d/priority.conf` |

## 当前配置

- 中间件为 `rmw_zenoh_cpp`，系统服务为 `inspection-zenoh.service`，运行在 cat 主机。
- Router 查询超时 `180000` ms；监听 `tcp/127.0.0.1:7447` 和 `tcp/192.168.168.111:7447`；连接本机 NX 的 `tcp/192.168.168.100:7447`；multicast、gossip 关闭；adminspace 启用。
- Session 查询超时 `180000` ms；连接 `tcp/127.0.0.1:7447`；监听 `tcp/192.168.168.111:0`；共享内存关闭；multicast 开启且接口固定为 `eth1`；gossip 关闭。
- 服务环境为 `ROS_DOMAIN_ID=24`，与各自 NX 和当前 ROS 业务进程保持一致。A/B 均使用 24；机器人之间的隔离依赖各自网络和 Zenoh 连接范围，不能仅靠相同或不同的 DOMAIN 数字判断。Router 的该环境变量也不构成跨域过滤规则。
- 优先级配置为 `Nice=-5`、`CPUWeight=600`。恢复时必须同时恢复 `priority.conf`，只复制主 service 文件会漏掉优先级。

2026-10-23 的验证记录显示 A/B 服务均为 active、enabled，DOMAIN 和优先级均已生效。这是当时状态，不代表实时查询结果。恢复后应在目标 cat 上检查：

```bash
systemctl show inspection-zenoh.service \
  -p ActiveState -p UnitFileState -p Environment -p Nice -p CPUWeight
```

上述 cat 服务与 NX 的 robot-launch 管理范围不同。NX 的应用服务操作必须通过 robot-launch，不能套用 cat 的 systemd 恢复方式。

## 来源与历史

逐文件 SHA-256、远端路径和本地快照来源见 [backup_record.json](backup_record.json)。本次依据已有迁移快照同步，未重新采集远端修改时间。

- `originals/20261023_before_domain24_sync/` 保存本次同步前的主 service（DOMAIN 28）和说明文档，供回溯。
- `originals/20261021T102843Z_before_d1m_a_overwrite/` 实际保留 router/session 两个文件，不是完整恢复包。
- `reports/20261015_eth1.md` 和 `originals/20261015_before_eth1/` 是此前 B 的历史材料，不代表当前配置。

总体范围和迁移限制见 [上级备份说明](../../README.md)。
