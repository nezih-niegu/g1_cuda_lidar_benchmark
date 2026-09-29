#!/usr/bin/env bash
# Prepare the source workspace; supports Humble/22.04 or Jazzy/24.04.
# Usage: bash scripts/setup_workspace.sh [humble|jazzy]
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REQUESTED="${1:-${ROS_DISTRO:-}}"
if [[ -n "$REQUESTED" && "$REQUESTED" != humble && "$REQUESTED" != jazzy ]]; then
  echo 'Only humble and jazzy are supported' >&2; exit 2
fi
# In a new shell, optional explicit distro overrides only an unsourced environment.
source "$DIR/detect_ros_distro.sh" "$REQUESTED"
if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  case "$ROS_DISTRO:${VERSION_ID:-}" in
    humble:22.04|jazzy:24.04) ;;
    *) echo "Warning: ROS $ROS_DISTRO on Ubuntu ${VERSION_ID:-unknown} is outside this project's primary binary-test matrix." >&2 ;;
  esac
fi
mkdir -p "$G1_WS/src"
SRC="$G1_WS/src/mujoco_ros2_control"
if [[ ! -d "$SRC/.git" ]]; then
  git clone --branch "$G1_UPSTREAM_BRANCH" https://github.com/dfki-ric/mujoco_ros2_control.git "$SRC"
else
  CURRENT="$(git -C "$SRC" branch --show-current)"
  if [[ "$CURRENT" != "$G1_UPSTREAM_BRANCH" ]]; then
    echo "Existing upstream checkout uses '$CURRENT', expected '$G1_UPSTREAM_BRANCH'." >&2
    echo "Use a fresh workspace or explicitly switch branches and rebuild; no automatic destructive checkout." >&2
    exit 2
  fi
fi
BENCH="$(cd "$DIR/.." && pwd)"
DEST="$G1_WS/src/g1_cuda_lidar_benchmark"
if [[ "$BENCH" != "$DEST" ]]; then
  if [[ -e "$DEST" ]]; then
    echo "Destination already exists: $DEST. Place/update this package there manually." >&2; exit 2
  fi
  cp -a "$BENCH" "$DEST"
fi
rosdep update
rosdep install --from-paths "$G1_WS/src" --ignore-src --rosdistro "$ROS_DISTRO" -y
# NVTX/CUDA requirements must be provisioned separately for GPU-enabled builds.
cd "$G1_WS"
colcon build --symlink-install --cmake-args "-DBUILD_CUDA=${BUILD_CUDA:-ON}" -DCMAKE_BUILD_TYPE=Release
echo "Build complete. In a clean terminal: source /opt/ros/$ROS_DISTRO/setup.bash && source $G1_WS/install/setup.bash"
