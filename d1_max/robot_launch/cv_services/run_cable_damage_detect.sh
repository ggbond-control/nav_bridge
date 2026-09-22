#!/usr/bin/env bash
set -euo pipefail

cd /home/robot/CV/cable-damege-detect
exec /home/robot/miniconda3/envs/cable/bin/gunicorn \
  --bind 0.0.0.0:8000 \
  --workers 1 \
  --worker-class gthread \
  --threads 4 \
  --timeout 120 \
  --access-logfile - \
  --error-logfile - \
  app_visual:app
