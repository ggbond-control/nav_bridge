#!/usr/bin/env bash
set -euo pipefail

cd /home/robot/CV/java
exec /usr/bin/java \
  -Xms512m \
  -Xmx512m \
  -Dspring.profiles.active=prod \
  -jar /home/robot/CV/java/algorithmCenter.jar
