# D1M-A / D1M-B 本地配置备份

本目录于 2026-10-23 按当天服务迁移完成后的快照同步，保存启动脚本、配置和恢复说明。本次同步只修改本地备份，未连接或操作机器人。

**2026-10-27 Zenoh 最新更新**：A 换板后先按 B 恢复缺失服务，随后按用户要求在两台 cat 关闭自动发现。最终 Router 只监听回环和机器人内网地址、固定连接所属 NX；Session 使用 `client` 模式，仅连接本机 Router，不监听入站端口；multicast/gossip 均关闭。文件超时为 `180000` ms，部分应用启动脚本的既有 600000 ms 覆盖保留。经用户明确允许，已重载 A/B 的 Zenoh 及相关 cat 服务；NX 未操作，各自雷达/IMU/cloud_merge 节点和点云数据正常。两狗接入同网后的隔离实测未做。当前以 [Zenoh 说明](zenoh/inspection_zenoh_d1m_b/README.md)为准；下方 2026-10-23 为历史基线，其他应用备份未重新采集。

cloud_merge 源码、CV 实际程序、模型和运行依赖在其他位置另有备份，本目录不重复收录。恢复前需从另存备份准备好对应程序和运行环境。

| 内容 | 入口 |
|---|---|
| cat 的 Zenoh 服务、Router/Session 和优先级 | [Zenoh 说明](zenoh/inspection_zenoh_d1m_b/README.md) |
| NX 的三个额外 robot-launch egg 和 A/B 启动快照 | [robot-launch 说明](robot_launch/README.md) |
| Java 和 GPU 检测启动脚本 | [CV 说明](robot_launch/cv_services/README.md) |
| cat 为所属 NX 提供的 HTTP/HTTPS 代理 | [代理说明](nx-net-proxy/README.md) |

## 机器对应关系

| 机器 | cat | NX 跳转 |
|---|---|---|
| A | `cat@121.41.75.74:20008` | `ssh -J cat@121.41.75.74:20008 robot@192.168.168.100` |
| B | `cat@121.41.75.74:20007` | `ssh -J cat@121.41.75.74:20007 robot@192.168.168.100` |

A/B 各自 cat 与 NX 的当前 ROS 通信均使用 DOMAIN 24；两套机器各用自己的内网和 Zenoh 连接。DOMAIN 24 本身不提供跨机器人隔离，恢复时必须保持连接指向所属 NX，不增加 A/B 之间的 Router 连接。

NX 直接运行在机器人上，应用服务必须通过 robot-launch 增量管理，不能为恢复三项应用而整体重启管理服务或覆盖原有配置。cat 的 Zenoh/proxy 则由 cat 的 systemd 管理。

## 2026-10-23 迁移验证基线

- A/B cat 的 Zenoh 均 active、enabled，`ROS_DOMAIN_ID=24`、`Nice=-5`、`CPUWeight=600`；Router/Session 查询超时 `180000` ms。
- B NX 已新增 17 cloud-merge、18 algorithm-center、19 cable-damage-detect；A 原有 19 项和 B 原有 16 项的运行 PID/状态保持不变，两台 NX 均未重启，robot-launch 总服务未重启。
- 点云输出约 8–10 Hz，frame_id 为 `d1m_base`；Java 监听 25682；检测监听 8000，使用 `cuda:0`、半精度和两个 v5 模型，本地图片推理通过。
- B cat 已部署并启用 `nx-net-proxy.service`，供 NX 检测服务外部模型调用使用。

这些是迁移时的验证结果，不是实时设备状态。来源和当前文件 SHA-256 见 [backup_record.json](backup_record.json)；本地迁移原始记录在 `/home/jazzy/d1m_migration_20261023/README.md`。

## 尚存限制与历史材料

B NX 在迁移核查时日期落后，导致外部 HTTPS 证书“尚未生效”；检测的本地 GPU 推理已通过，外部大模型调用仍受授时影响。未修改授时或绕过 TLS。

B 既有 `/opt/inspection_bringup/run_navigation.sh`、`run_inspection_system.sh`、`run_inspection_hardware.sh` 仍写 CycloneDDS / DOMAIN 77；对应三个系统服务当时 inactive/disabled，本次迁移未修改或启用它们。以后启用前需要另行统一通信配置。

`originals/`、`reports/`、`legacy/` 和 `before-cloud-merge` 为历史材料。A/B 完整 robot-launch 快照分别保存，通用文件名 `robot_launch/robot_supervisor/robot_egg_launch.yaml` 保存 A 的内容。恢复时仅合并所需额外条目。

充电相关文件、WIFI 服务和代理实现未因本次同步修改；代理文档仅纠正地址并补充当前依赖说明。
