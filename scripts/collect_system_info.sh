#!/usr/bin/env bash
set -u
OUT="${1:-system_info.txt}"
{
  date -Ins
  uname -a
  echo '--- ROS ---'; printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID 2>/dev/null || true
  echo '--- OS ---'; grep -E '^(NAME|VERSION_ID)=' /etc/os-release 2>/dev/null || true
  echo '--- UPSTREAM BRANCH ---'; git -C "${UPSTREAM_PATH:-$HOME/g1_ws/src/mujoco_ros2_control}" branch --show-current 2>/dev/null || true
  echo '--- NVIDIA ---'; nvidia-smi --query-gpu=name,driver_version,memory.total,power.limit --format=csv 2>/dev/null || true
  echo '--- CUDA ---'; nvcc --version 2>/dev/null || true
  echo '--- SOURCE REVISION ---'; git -C "${UPSTREAM_PATH:-$HOME/g1_ws/src/mujoco_ros2_control}" rev-parse HEAD 2>/dev/null || true
  echo '--- CPU ---'; lscpu 2>/dev/null | grep -E 'Model name|CPU\(s\)' || true
} > "$OUT"
echo "Wrote $OUT"
