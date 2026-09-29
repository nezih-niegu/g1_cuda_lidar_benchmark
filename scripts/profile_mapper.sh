#!/usr/bin/env bash
# Profile mapper-only. In a second terminal, replay the same rosbag during capture.
# Usage: profile_mapper.sh naive|optimized /absolute/output/prefix [nsys|ncu]
set -euo pipefail
if [[ $# -lt 2 ]]; then echo "Usage: $0 naive|optimized OUT_PREFIX [nsys|ncu]" >&2; exit 2; fi
MODE="$1"; OUT="$2"; TOOL="${3:-nsys}"
case "$MODE" in naive|optimized) ;; *) echo 'Choose naive or optimized' >&2; exit 2;; esac
case "$TOOL" in
  nsys)
    exec nsys profile --trace=cuda,nvtx,osrt --sample=none --delay=5 --duration=30 \
      -o "$OUT" ros2 run g1_cuda_lidar_benchmark lidar_mapper --ros-args \
      -p "backend:=$MODE" -p 'use_sim_time:=true' -p "csv_path:=${OUT}.csv"
    ;;
  ncu)
    exec ncu --target-processes all --set basic --launch-count 10 -o "$OUT" \
      ros2 run g1_cuda_lidar_benchmark lidar_mapper --ros-args \
      -p "backend:=$MODE" -p 'use_sim_time:=true' -p "csv_path:=${OUT}.csv"
    ;;
  *) echo 'Tool must be nsys or ncu' >&2; exit 2;;
esac
