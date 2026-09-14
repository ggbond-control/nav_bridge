# D1 Max `cloud_merge` / `robot-launch` 配置备份

本目录是从 D1 Max 导航主机 `robot@192.168.168.100` 导出的配置备份。备份对象原先位于导航主机的 `~/Workspace/driver_ws` 和 `~/.robot_egg_launch.yaml`，用于固件升级后恢复；固件升级可能删除第三方 ROS 包、启动脚本和 `robot-launch` egg 配置。

## 目录内容

```text
cloud_merge/
  package.xml
  CMakeLists.txt
  src/cloud_merge_node.cpp
  launch/cloud_merge.launch.py
  config/cloud_merge.yaml
  rviz/cloud_merge.rviz
start_cloud_merge.sh
robot_supervisor/
  robot_egg_launch.yaml                 # 当前配置，含 cloud-merge
  robot_egg_launch.yaml.before-cloud-merge # 添加 cloud-merge 前的基线
```

当前 cloud merge 的主要参数为：

- 输入：`/front_lidar`、`/rear_lidar`
- 原始合并输出：`/lidar/airy/origin`
- 体素降采样输出：`/lidar/airy`
- 基座坐标系：`d1m_base`
- 前后雷达坐标系：`rslidar_head`、`rslidar_tail`

## 固件升级后恢复

以下命令在导航主机（NX）执行。NX 需要先通过 cat 主机登录：

```bash
ssh cat@10.0.40.226
ssh robot@192.168.168.100
```

### 1. 恢复源码

将本目录中的 `cloud_merge` 复制到导航主机：

```bash
mkdir -p ~/Workspace/driver_ws/src
scp -r <备份目录>/cloud_merge ~/Workspace/driver_ws/src/
```

如果备份目录位于代码仓库，可在开发机执行（通过 cat 跳转时使用 ProxyJump 或等价的 ProxyCommand）：

```bash
scp -r d1_max/robot_launch/cloud_merge \
  robot@192.168.168.100:~/Workspace/driver_ws/src/
```

确认目录结构为 `~/Workspace/driver_ws/src/cloud_merge/{package.xml,CMakeLists.txt,src,launch,config,rviz}`。

### 2. 编译并安装

```bash
cd ~/Workspace/driver_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cloud_merge --symlink-install
source install/setup.bash
```

如果工作区中没有 `colcon` 或依赖包，先按设备提供的 ROS 环境恢复依赖；不要把本备份链接到开发机的绝对路径。

### 3. 恢复启动脚本

```bash
mkdir -p ~/Workspace/driver_ws/bin
cp <备份目录>/start_cloud_merge.sh ~/Workspace/driver_ws/bin/
chmod +x ~/Workspace/driver_ws/bin/start_cloud_merge.sh
```

脚本内容等价于：

```bash
#!/usr/bin/env bash
set -e
source /opt/runtime/env.bash
source /home/robot/Workspace/driver_ws/install/setup.bash
exec ros2 launch cloud_merge cloud_merge.launch.py use_rviz:=false
```

若工作区路径改变，只需同步修改脚本中的 `source` 路径。

### 4. 将 cloud merge 加回 `robot-launch`

`robot-launch` 使用用户目录下的 `~/.robot_egg_launch.yaml`。在 `eggs:` 下加入以下条目（`id` 必须与现有条目不重复）：

```yaml
  cloud-merge:
    command: bash /home/robot/Workspace/driver_ws/bin/start_cloud_merge.sh
    name: cloud-merge
    id: 17
    state:
      status: Running
      start_time: null
      try_count: 0
      error: ''
      pid: 0
    paths:
      stdout: /home/robot/robot_launch_log/cloud-merge.stdout
      stderr: /home/robot/robot_launch_log/cloud-merge.stderr
```

建议先保留升级前的配置副本：

```bash
cp ~/.robot_egg_launch.yaml ~/.robot_egg_launch.yaml.before-cloud-merge
```

配置写入后，重启 `robot-launch` 管理的 egg：

```bash
robot-launch list
robot-launch collect cloud-merge
# 若 cloud-merge 已存在但停止：
robot-launch start cloud-merge
# 若已运行且需要重新加载脚本：
robot-launch restart cloud-merge
```

不同固件版本的 `robot-launch` 子命令可能略有差异；以 `robot-launch --help` 为准。`robot-launch server` 应由系统已有服务启动，不要重复启动多个 server。

### 5. 手动启动（不接入 robot-launch）

用于首次验证或排查配置时：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Workspace/driver_ws/install/setup.bash
bash ~/Workspace/driver_ws/bin/start_cloud_merge.sh
```

验证话题：

```bash
ros2 node list | grep cloud_merge
ros2 topic list | grep -E '/(front_lidar|rear_lidar|lidar/airy)'
ros2 topic hz /lidar/airy
ros2 topic echo --once /lidar/airy/header
```

确认输入雷达驱动已启动、输出频率稳定且 `frame_id` 为 `d1m_base`。若输出为空，优先检查输入话题名称、时间戳同步容差和 TF/标定参数，而不是修改 `robot-launch`。

## 固件升级后的推荐顺序

1. 升级完成并重启 NX。
2. 恢复 `cloud_merge` 源码到 `~/Workspace/driver_ws/src/cloud_merge`。
3. 编译工作区并恢复 `start_cloud_merge.sh`。
4. 从本备份的 `robot_egg_launch.yaml` 中复制 `cloud-merge` 条目到新的 `~/.robot_egg_launch.yaml`；不要整体覆盖新固件生成的配置，以免丢失新版本 egg。
5. `robot-launch collect cloud-merge` 或 `restart cloud-merge`。
6. 按上面的 ROS 话题命令验证；确认无误后再启动导航算法。

## 备份完整性与版本说明

本备份保留了当前设备上的源码和标定参数，不代表新固件一定兼容。升级后若雷达驱动更换了话题名、坐标系或 PointCloud2 字段布局，应只修改 `config/cloud_merge.yaml` 中对应参数，并保留原始备份以便回滚。

