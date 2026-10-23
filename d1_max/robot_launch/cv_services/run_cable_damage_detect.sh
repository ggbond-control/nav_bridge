#!/usr/bin/env bash
set -euo pipefail

# ===== GPU 环境 =====
# 让 torch 找到 pip 装的 nvidia 库（cusparselt 等）
export LD_LIBRARY_PATH="/home/robot/miniconda3/envs/cable-gpu/lib/python3.10/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH"
# 屏蔽 ~/.local 干扰
export PYTHONNOUSERSITE=1
# 使用 GPU
export DEVICE=cuda:0
export HALF=1

# ===== 公司代理（调大模型 API 用）=====
export http_proxy=http://192.168.168.111:18080
export https_proxy=http://192.168.168.111:18080
export no_proxy=localhost,127.0.0.1,192.168.0.0/16,10.0.0.0/8,172.16.0.0/12
export NO_PROXY="$no_proxy"

cd /home/robot/CV/cable-damege-detect

# ===== 启动（★ gunicorn 从 cable → cable-gpu）=====
exec /home/robot/miniconda3/envs/cable-gpu/bin/gunicorn \
  --bind 0.0.0.0:8000 \
  --workers 1 \
  --worker-class gthread \
  --threads 4 \
  --timeout 120 \
  --access-logfile - \
  --error-logfile - \
  app_visual:app
