#!/usr/bin/env bash
# Usage: run_three_modes.sh /absolute/path/to/rosbag /absolute/output/dir
# Run outside the simulator for fair GPU comparisons. Requires sourced workspace.
set -euo pipefail
if [[ $# -ne 2 ]]; then echo "Usage: $0 BAG_DIRECTORY OUTPUT_DIRECTORY" >&2; exit 2; fi
BAG="$(realpath "$1")"
OUT="$2"
mkdir -p "$OUT"
OUT="$(realpath "$OUT")"
if [[ ! -f "$BAG/metadata.yaml" ]]; then echo "Missing rosbag metadata.yaml" >&2; exit 2; fi
: "${ROS_DISTRO:?Source ROS 2 Humble workspace first}"
if [[ "$ROS_DISTRO" != "humble" ]]; then echo "Designed for ROS 2 Humble" >&2; exit 2; fi
RUNS="${RUNS:-5}"
WARMUP="${WARMUP:-100}"
DEADLINE_MS="${DEADLINE_MS:-100}"
MAP_FRAME="${MAP_FRAME:-}"
SLEEP_READY="${SLEEP_READY:-3}"
stop_mapper() {
    if [[ -n "${MAPPER_PID:-}" ]]; then
        kill -INT "$MAPPER_PID" 2>/dev/null || true
        wait "$MAPPER_PID" 2>/dev/null || true
        MAPPER_PID=""
    fi
}
trap stop_mapper EXIT INT TERM
for run in $(seq 1 "$RUNS"); do
    for backend in cpu naive optimized; do
        CSV="$OUT/${backend}_run${run}.csv"
        echo "=== run $run, backend $backend ==="
        ros2 run g1_cuda_lidar_benchmark lidar_mapper --ros-args \
            -p "backend:=$backend" -p "csv_path:=$CSV" \
            -p 'use_sim_time:=true' -p "map_frame:=$MAP_FRAME" \
            >"$OUT/${backend}_run${run}.log" 2>&1 &
        MAPPER_PID=$!
        sleep "$SLEEP_READY"
        if ! kill -0 "$MAPPER_PID" 2>/dev/null; then
            cat "$OUT/${backend}_run${run}.log" >&2
            echo "Mapper failed to start; check CUDA installation and logs." >&2
            exit 1
        fi
        # Exactly one producer: rosbag. MuJoCo and relay must be stopped.
        if [[ "${PLAY_CLOCK:-0}" == "1" ]]; then
            ros2 bag play "$BAG" --clock
        else
            ros2 bag play "$BAG"
        fi
        sleep 1
        stop_mapper
        if [[ ! -s "$CSV" ]]; then echo "No CSV rows: $CSV" >&2; exit 1; fi
    done
done
python3 "$(dirname "$0")/summarize.py" "$OUT"/*_run*.csv \
    --warmup "$WARMUP" --deadline-ms "$DEADLINE_MS" \
    --out "$OUT/comparison.csv"
echo "Comparison: $OUT/comparison.csv"
