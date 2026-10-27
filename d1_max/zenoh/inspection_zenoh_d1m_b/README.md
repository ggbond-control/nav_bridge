# D1M-A / D1M-B Zenoh 固定连接配置

本目录保存 **2026-10-27 关闭自动发现并应用到两狗后的 cat 配置**。目标是保留每只狗内部 cat ↔ NX 的 ROS 2 通信，同时避免经 Wi-Fi/5G 自动发现另一只狗。两狗目前仍使用 DOMAIN 24；本次没有修改 NX。

| 机器 | cat 登录方式 | NX 跳转（地址参考，本次未操作） |
|---|---|---|
| A | `ssh -p 20008 cat@121.41.75.74` | `ssh -J cat@121.41.75.74:20008 robot@192.168.168.100` |
| B | `ssh -p 20007 cat@121.41.75.74` | `ssh -J cat@121.41.75.74:20007 robot@192.168.168.100` |

各自的 `.111`（cat）与 `.100`（NX）属于各自机器人内网，相同地址不代表共用设备。每台 Router 连接自己 NX，未增加 A/B Router 之间的连接。

## 部署文件和通信路径

| 本地文件 | cat 部署路径 |
|---|---|
| `inspection-zenoh.service` | `/etc/systemd/system/inspection-zenoh.service` |
| `inspection-zenoh.service.d/priority.conf` | `/etc/systemd/system/inspection-zenoh.service.d/priority.conf` |
| `router_config.json5` | `/home/cat/Workspace/zenoh_ws/router_config.json5` |
| `session_config.json5` | `/home/cat/Workspace/zenoh_ws/session_config.json5` |

```text
本狗 cat 的 ROS 节点（client）
  → tcp/127.0.0.1:7447（本狗 cat Router）
    → tcp/192.168.168.100:7447（本狗 NX Router）
```

- **Router**：`mode=router`，只监听 `127.0.0.1:7447` 和 `192.168.168.111:7447`；固定连接所属 NX；multicast 和 gossip 均关闭，并关闭 multicast scout 响应。不会在 Wi-Fi/5G 地址或 IPv6 通配地址监听。
- **Session**：`mode=client`，只连接本机 `127.0.0.1:7447`；`listen.endpoints=[]`，不接受入站连接；multicast 和 gossip 均关闭，并关闭 multicast scout 响应。ROS 节点发现和话题通信仍通过固定 Router 路径进行。
- Router/Session 文件的查询超时恢复为原备份的 `180000` ms；共享内存关闭，Router adminspace 保持启用。现机部分启动脚本另有 `ZENOH_CONFIG_OVERRIDE=queries_default_timeout=600000`，这些进程的有效查询超时仍为 600000 ms；该覆盖只改变超时，不重新开启自动发现，本次未改启动脚本。
- 系统服务保持 `ROS_DOMAIN_ID=24`、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`、`Nice=-5`、`CPUWeight=600`，并启用开机自启。

本次以原备份的限定监听和固定 NX 连接为基础，进一步关闭自动发现。Session 采用 client 模式，使所有本机节点通过 Router 通信；在当前版本上，保留 peer 模式同时关闭发现的试验导致本机节点互相可见性不完整，因此没有作为最终版本。

## 生效方式与验证

用户明确允许后，分别停止两台 cat 原先运行的 `inspection-system`、`inspection-navigation`、`inspection-hardware`，重载 Zenoh，再恢复这些服务；没有重启整机或操作 NX 服务，也没有发送导航启动请求。既有导航任务会因服务重启中止，不能把服务恢复理解为自动恢复原导航任务。

只改配置文件不会让已有 ROS Session 自动重读；今后更新时需在合适窗口重启依赖该 Session 的相关进程。NX 业务继续由 robot-launch 管理，不应套用 cat 的 systemd 操作。

验收结果：

- 本地和 A/B 的四个部署文件 SHA-256 一致，配置均通过板上实际 Zenoh 库解析。
- 两台 cat 的 Zenoh、导航入口、巡检和硬件服务均 active；Zenoh active/enabled，优先级 -5/600 已生效。
- Router 仅有上述两个监听地址，已建立到所属 NX `192.168.168.100:7447` 的连接；cat ROS Session 没有新增 TCP 监听。已有媒体 HTTP、相机和 MQTT 业务网络连接未修改，不属于 Zenoh 监听。
- A/B 本机的巡检节点、`navigation_supervisor`、NX 的雷达、IMU、`cloud_merge` 均可见；`/navigation_bringup/start` 可发现，未调用它。
- 24 秒订阅 `/lidar/airy`：A 收到 240 帧，B 收到 241 帧，frame_id 均为 `d1m_base`。
- 临时探针各自可见、另一只狗的探针不可见。但两狗当前位于不同楼层，未验证是否处于同一可达网络，**不能用这次结果证明接入同网后的隔离效果**。本次确认的是配置约束、实际监听范围和本狗 NX 数据正常；同网实测另行安排。

隔离前提是两套机器人内网保持独立，全部相关 ROS 进程使用本目录配置且没有其他跨狗 Router/桥接连接。DOMAIN 24 不是安全边界；本次未增加防火墙、鉴权或核改 NX 配置。

只读检查命令（在目标 cat 的 ROS 环境执行）：

```bash
systemctl show inspection-zenoh.service \
  -p ActiveState -p UnitFileState -p Environment -p Nice -p CPUWeight
ss -ltn | grep ':7447'
ss -tn | grep '192.168.168.100:7447'
ros2 node list --no-daemon --spin-time 8
ros2 topic hz /lidar/airy
```

## 来源、回退和历史

逐文件 SHA-256、部署路径及验证状态见 [backup_record.json](backup_record.json)。本地操作、配置快照和验证原始输出位于 `/home/jazzy/d1m_zenoh_isolation_20261027/`。

两台 cat 的修改前备份均在 `/home/cat/d1m-migration-backups/20261027-before-static-isolation/`。B 另有 `20261027-before-static-isolation-client-mode/`，保存验证期间的中间 peer 版本；如需回到本次操作前，应使用前一个备份。回退也需配合相关 Session 进程重载，不能只恢复文件。

- `originals/20261027_before_static_isolation/`：本次关闭发现前的本地四个部署文件及文档、校验记录。
- `originals/20261027_before_B_reference_sync/`：此前按 B 现机同步前的本地备份，含 180000 ms 和 eth1 组播的旧配置。
- 其他 `originals/`、`reports/` 保存更早材料，原样保留，不代表当前部署。

总体范围见 [上级备份说明](../../README.md)。
