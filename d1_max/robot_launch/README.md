# D1 Max NX 的 robot-launch 配置备份

本目录保存 2026-10-23 迁移完成后适用于 D1M-A/B 的启动脚本、三个额外 egg 的独立配置和各自的启动 YAML 快照。NX 直接运行在机器人上，应用的注册、启停和重启必须通过 robot-launch；不要重启整个管理服务或手动再启动一份应用。

cloud_merge 源码、标定配置以及 CV 程序、模型和运行依赖在其他位置备份，本目录不重复收录。恢复这些材料后，再使用本目录的启动脚本和 egg 配置。

## 目录内容

```text
start_cloud_merge.sh
eggs/                                  # 三个单 egg YAML，供 collect 使用
  cloud-merge.yaml
  algorithm-center.yaml
  cable-damage-detect.yaml
cv_services/                           # CV 启动脚本和部署说明
robot_supervisor/
  robot_egg_launch.yaml                 # A 的 19 项快照，兼容旧文件名
  snapshots/20261023/
    D1M-A.robot_egg_launch.yaml          # A 的启动配置快照
    D1M-B.robot_egg_launch.yaml          # B 的启动配置快照
  robot_egg_launch.yaml.before-cloud-merge
  originals/20261023_before_extra_eggs_sync/robot_egg_launch.yaml
```

完整 YAML 只供核对。A/B 原有条目状态存在差异，例如 `robot-forward`：A 为 Stopped，B 为 Running。恢复时保留目标机器或新固件的原有条目，仅补所需的 17–19，不能用 A 的完整文件覆盖 B，也不能用快照覆盖新固件配置。快照内的 PID 0、启动时间等不是实时运行状态。

## 登录目标 NX

在开发机选择其中一条，NX 必须经所属机器人跳转：

```bash
# A 的 NX
ssh -J cat@47.99.202.196:20004 robot@192.168.168.100
# B 的 NX
ssh -J cat@10.0.40.237 robot@192.168.168.100
```

## 恢复程序和启动脚本

先从另存备份恢复 cloud_merge 到 `~/Workspace/driver_ws/src/cloud_merge`，按目标 NX 的 ROS Humble 环境编译；已验证可用的 ARM 安装产物可复用。若需编译：

```bash
cd ~/Workspace/driver_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cloud_merge --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
file build/cloud_merge/cloud_merge_node
```

可执行文件应是 ARM aarch64 ELF，不能是空文件或开发机的 x86 文件。将本目录 `start_cloud_merge.sh` 放到 `/home/robot/Workspace/driver_ws/bin/` 并赋予执行权限。脚本直接 source isolated install 中的 `install/cloud_merge/share/cloud_merge/local_setup.bash`，并设置包前缀，避免仅 source 空的顶层 setup。

CV 程序、GPU 环境和四个脚本的恢复前提见 [cv_services/README.md](cv_services/README.md)。所有程序和配置就绪后再注册 egg。

## 通过 robot-launch 增量恢复

以下命令在 NX 的 robot 用户会话中执行。先查看 `robot-launch list` 和 `/home/robot/.robot_egg_launch.yaml`，确认名称及 ID 没有冲突。17、18、19 分别对应 cloud-merge、algorithm-center、cable-damage-detect；已存在的 egg 不重复 collect。

先保存目标机原配置：

```bash
cp -p /home/robot/.robot_egg_launch.yaml \
  "/home/robot/.robot_egg_launch.yaml.before-extra-eggs-$(date +%Y%m%dT%H%M%S)"
robot-launch list
```

将本目录的 `eggs/` 放到 NX 的 `/home/robot/robot-launch-backup/eggs/`。对确认缺失的项逐项执行；collect 会安排启动，执行一项并确认正常后再继续下一项：

```bash
robot-launch collect /home/robot/robot-launch-backup/eggs/cloud-merge.yaml
robot-launch egg 17
robot-launch collect /home/robot/robot-launch-backup/eggs/algorithm-center.yaml
robot-launch egg 18
robot-launch collect /home/robot/robot-launch-backup/eggs/cable-damage-detect.yaml
robot-launch egg 19
```

`collect` 的参数是**单个 egg 的 YAML 文件路径**，不是 egg 名称，也不是顶层含 `eggs:` 的完整文件。固件版本变化时先查看 `robot-launch --help` 及对应子命令帮助。

注册后核对 `/home/robot/.robot_egg_launch.yaml`：三个条目必须持久化在顶层 `eggs:` 下，各出现一次；如当前版本未自动保存，仅将缺失条目合并进去，不改原有条目。现有 egg 更新脚本后，如需重启，只通过 robot-launch 重启对应 ID，并按该版本帮助执行。不要重启整个 `robot-launch.service`，也不要另起 `robot-launch server`。

## 验证

```bash
robot-launch list
robot-launch egg 17
robot-launch egg 18
robot-launch egg 19
```

点云检查需要使用目标 NX 的 ROS 环境，保持 `ROS_DOMAIN_ID=24`、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`：

```bash
source /opt/runtime/env.bash
source /opt/ros/humble/setup.bash
source ~/Workspace/driver_ws/install/cloud_merge/share/cloud_merge/local_setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic hz /lidar/airy
ros2 topic echo --once /lidar/airy/header
```

当前参数：输入 `/front_lidar`、`/rear_lidar`，原始输出 `/lidar/airy/origin`，降采样输出 `/lidar/airy`，基座 `d1m_base`，前后雷达坐标系 `rslidar_head`、`rslidar_tail`。迁移时输出约 8–10 Hz，frame_id 为 `d1m_base`。输出为空时检查输入、时间同步和标定配置。

CV 的端口、健康检查和代理/授时限制见 [CV 说明](cv_services/README.md)；整体验证基线见 [上级说明](../README.md)。历史目录及 `before-cloud-merge` 原样保留。

## bot 主机 WIFI 设置（原有参考）

```bash
wpa_passphrase FHZN FHZN1234 > ~/FHZN_WIFI.conf
```

本次本地备份同步未调整 WIFI 服务。
