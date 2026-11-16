# D1 MAX 登阶模式速度实测

`stair_speed_probe.cpp` 是独立 SDK 实机程序，绕过 nav_bridge 的速度换算。
它只调用 `Stair()` 和 `Move()` 切换姿态、执行运动，**不调用 `SetSpeed()`**。
按当前补充说明，Stair 不受速度档位影响；CSV 中的 `speed_level` 仅为状态记录。

程序不自动起立、清除急停或遍历全速区间。实机应先处于静止站立状态；先在平整开阔
场地测模式本身的映射，再考虑楼梯环境。现场人员负责观察和急停。运行前停止
nav_bridge、导航及其他 SDK 控制程序，避免同时下发运动命令；`control_source=SDK`
不能区分两个 SDK 客户端。

## 编译

在工作空间根目录，使用当前工程的 D1 MAX 构建选项：

```bash
cd /home/jazzy/drive_ws
colcon build --packages-select nav_bridge --cmake-args -DNAV_BRIDGE_BUILD_D1_MAX=ON
source install/setup.bash
ros2 run nav_bridge d1_max_stair_speed_probe --help
```

已有 CMake 构建目录时也可以直接构建目标：

```bash
cmake -S src/nav_bridge -B build/nav_bridge
cmake --build build/nav_bridge --target d1_max_stair_speed_probe --parallel 2
```

实机程序不会由 `ctest` 执行。`d1_max_stair_probe_test` 使用替代 SDK 方法做离线验证，
不连接机器人。

## 预览和运行

`--sample` 三个值的顺序为 **前后、横向、旋转**，均为 `[-1,1]` 的 SDK 输入，
不是 m/s 或 rad/s。程序内部会按 SDK 要求转换参数顺序为 `Move(横向, 前后, 旋转)`。
正负方向沿用 SDK：正数为前进、左移、左转。

默认只打印计划，不建立 SDK 连接、不生成 CSV：

```bash
./build/nav_bridge/d1_max_stair_speed_probe --sample 0.1,0,0
```

现场条件确认后，以下命令才会实际连接并运动；`--host` 按机器人实际地址填写：

```bash
./build/nav_bridge/d1_max_stair_speed_probe \
  --execute --host 192.168.234.1 --port 8082 \
  --sample 0.1,0,0 --settle 1 --duration 2 --pause 1 \
  --output stair_forward_01.csv
```

一次采样会先持续发送该输入 `settle` 秒，再保持输入测量 `duration` 秒，
然后发送零速 `pause` 秒。**非零运动总时长为 `settle + duration`**。
Move 大约每 50 ms 发送一次，同步调用自身也可能增加间隔。
每段结束必须收到两路速度反馈均接近零，才进入下一段。
`settle/duration` 范围为 0.2–10 秒，`pause` 为 1–10 秒。

通过替换 `--sample` 分别测试不同方向；以下命令均为预览：

```bash
# 后退
./build/nav_bridge/d1_max_stair_speed_probe --sample -0.1,0,0
# 左、右横移（依次执行，两段之间停稳）
./build/nav_bridge/d1_max_stair_speed_probe --sample 0,0.1,0 --sample 0,-0.1,0
# 左、右旋转
./build/nav_bridge/d1_max_stair_speed_probe --sample 0,0,0.1 --sample 0,0,-0.1
# 前进叠加横移、旋转
./build/nav_bridge/d1_max_stair_speed_probe --sample 0.1,0.1,0 --sample 0.1,0,0.1
```

每次从小输入验证后，再手动选择更大的输入。程序不假设前进/后退、左/右映射对称，
也不预设 Stair 的速度上限。当前本地桥接采用的低速换算不参与本程序。

## 数据和退出

程序开启 `OnSpeedData`（配置 50 Hz）和 `OnMcData`，CSV 每行标记数据来源：

| source | 前后/横向速度 | 转向角速度 |
| --- | --- | --- |
| `speed` | `SpeedData.x/y` | `SpeedData.yaw` |
| `mc_body` | `MotionData.v_body[0/1]` | `MotionData.omega_body[2]` |
| `robot_state` | `RobotState.speed.line/translation` | `RobotState.speed.angle` |

速度单位为 m/s，角速度为 rad/s。`mc_body` 的角速度绕机体 Z 轴；在倾斜楼梯上
不必然等于世界坐标系 yaw 角的变化率。两路反馈是否独立测量、精度如何，SDK 文档未说明。

CSV 包含本机单调时间、Unix 纳秒时间、采样编号、阶段、请求的 Move 百分比、反馈速度、
运动模式、控制源和原始运动数据时间戳。`requested_*_pct` 是该阶段请求的输入，
不代表机器人已执行确认；模式与控制源来自最近的 `RobotState`。
`robot_timestamp_ns` 仅 `mc_body` 行有值，其他来源填 0。

控制台分别输出 `speed` 和 `mc_body` 在 `measure` 阶段的样本数和均值/最小值/最大值。
这里的 `settle` 只是排除初始过渡的时间，**不自动证明已达稳定速度**；需要检查 CSV 的曲线，
必要时延长采样。均值也不是最大速度上限的证明。

速度/运动反馈超过 0.5 秒未更新、机器人状态超过 2.5 秒未更新、控制丢失、退出 Stair、
急停、严重故障、发送失败或按 Ctrl+C 时，会终止测试并尽力发送零速、释放控制权。
网络断开、进程被强杀等情况下，程序不能保证停车，现场急停仍是必要措施。
正常结束后保持 Stair 姿态，不自动卧倒或切换其他姿态。

不指定 `--output` 时生成带时间戳的 CSV；已有的可读输出文件会拒绝覆盖。
异常退出保留已写入的数据，退出码非零，不能把其中未完成的采样当作完整测试结果。
程序不会自动停止机器人上其他业务进程，也不会自动部署或修改现有速度映射。
