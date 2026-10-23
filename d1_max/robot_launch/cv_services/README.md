# CV 服务的 robot-launch 配置备份

本目录的四个启动脚本以 2026-10-23 A 现机版本为准，该版本已迁移并验证于 B：

- `algorithm-center`：ID 18，Java 服务，TCP 25682。
- `cable-damage-detect`：ID 19，检测 HTTP 服务，TCP 8000。

Java JAR、业务配置、检测程序、模型、GPU 环境和依赖另有备份，本目录只保存启动配置。运行配置中的 API 密钥应按另存备份恢复，不写入此目录。

## 启动方式与运行前提

robot-launch 由 root 运行，`start_*.sh` 使用 `setpriv` 切换为 robot，并显式设置 HOME、XDG 缓存/配置目录，避免应用写入 `/root`。`run_algorithm_center.sh` 以前台 `exec java` 运行；原程序的 `app.sh` 使用后台 nohup，不能作为 egg 命令，否则管理器不能跟踪实际 JVM。

检测脚本使用绝对路径 `/home/robot/miniconda3/envs/cable-gpu/bin/gunicorn`，无需激活 Conda base；恢复 GPU 环境时必须保留该前缀。脚本设置 CUDA 库路径、`PYTHONNOUSERSITE=1`、`DEVICE=cuda:0` 和 `HALF=1`。Gunicorn 使用一个 worker、四个线程，避免重复加载模型占用 Orin 显存。

2026-10-23 验证环境为 Python 3.10.21、PyTorch `2.5.0a0+872d972e41.nv24.08`、TorchVision 0.20.0、CUDA 12.6。模型为 `cable-v5.pt`、`cabledamage-v5.pt`。这些是另存运行环境的核对基线，不是本目录提供的安装包。

外部模型调用使用 HTTP/HTTPS 代理 `http://192.168.168.111:18080`，需所属 cat 主机的 `nx-net-proxy.service` 正常，见 [代理说明](../../nx-net-proxy/README.md)。

## 部署与注册

1. 从另存备份恢复 Java/检测程序、模型、配置及 GPU 环境。
2. 将本目录四个 shell 脚本复制到 NX 的 `/home/robot/CV/bin/`，赋予执行权限。
3. 通过 robot-launch 检查现有条目和端口占用。已有 ID 18/19 或同名服务时先核对，不重复注册；端口有其他进程时先确认来源，不直接批量杀进程。
4. 按 [上级恢复步骤](../README.md)备份启动 YAML，将 `../eggs/` 复制到 NX 的 `/home/robot/robot-launch-backup/eggs/`。对确认缺失的服务逐项执行：

```bash
robot-launch collect /home/robot/robot-launch-backup/eggs/algorithm-center.yaml
robot-launch egg 18
robot-launch collect /home/robot/robot-launch-backup/eggs/cable-damage-detect.yaml
robot-launch egg 19
```

`robot_egg_entries.yaml` 是供合并到完整启动 YAML 的参考片段，不能直接交给 collect。注册后核对 `/home/robot/.robot_egg_launch.yaml` 已持久化两项，保留原有条目。应用启停、重启均通过 robot-launch 管理；不要重启整个 NX 管理服务。

## 验证与已知限制

```bash
robot-launch egg 18
robot-launch egg 19
curl -fsS http://127.0.0.1:8000/health
ss -ltnp | grep -E ':(8000|25682)\b'
```

检测健康状态应为 `ok`、`device=cuda:0`、`half=true`，并加载上述 v5 模型。Java 根路径返回 HTTP 404 是应用原有行为，没有 `/health` 或 `/actuator/health`，通过管理器状态、robot 用户进程和监听端口核对。

迁移时 CUDA 运算、CUDA NMS 和跳过外部大模型的图片请求 `/predict?llm_skip=1&minimal=1` 已通过。这验证了本地推理链路，不代表外部 API 可用或识别准确率已评估。

2026-10-23 核查时 B NX 日期仍为 2026-02-24，HTTPS 证书检查出现“尚未生效”，外部大模型调用受限。NX 受 PPS 和内部授时源管理；本次迁移及本地备份同步未修改授时，也未关闭 TLS 验证。`llm.enabled=true` 只表示功能启用，不能当作 API 调用成功。另有 torchvision.io 缺少 `libnvjpeg.so.12` 的提示，当时 OpenCV 读图和本地 CUDA 推理已通过。
