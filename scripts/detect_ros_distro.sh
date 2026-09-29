#!/usr/bin/env bash
# Source this file to select an installed ROS 2 distribution.
# Usage: source scripts/detect_ros_distro.sh [humble|jazzy]
_g1_requested="${1:-${ROS_DISTRO:-}}"
if [[ -z "$_g1_requested" ]]; then
  if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    case "${VERSION_ID:-}" in
      22.04) _g1_requested=humble ;;
      24.04) _g1_requested=jazzy ;;
    esac
  fi
fi
case "$_g1_requested" in
  humble|jazzy) ;;
  *) echo "Choose humble or jazzy: source scripts/detect_ros_distro.sh jazzy" >&2; return 2 2>/dev/null || exit 2 ;;
esac
if [[ ! -f "/opt/ros/$_g1_requested/setup.bash" ]]; then
  echo "ROS 2 $_g1_requested not installed at /opt/ros/$_g1_requested" >&2
  return 2 2>/dev/null || exit 2
fi
# Detect an accidental cross-distro sourced environment.
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$_g1_requested" ]]; then
  echo "ROS_DISTRO=$ROS_DISTRO is already sourced; open a clean shell to select $_g1_requested" >&2
  return 2 2>/dev/null || exit 2
fi
# ROS setup scripts may refer to optional environment variables before they
# have been initialized (e.g. AMENT_TRACE_SETUP_FILES). Temporarily turn off
# nounset when sourcing them, then restore the caller's original state.
_g1_had_nounset=0
case $- in *u*) _g1_had_nounset=1; set +u ;; esac
# shellcheck disable=SC1090
source "/opt/ros/$_g1_requested/setup.bash"
_g1_source_status=$?
if [[ $_g1_had_nounset -eq 1 ]]; then set -u; fi
if [[ $_g1_source_status -ne 0 ]]; then
  echo "Unable to source ROS 2 $_g1_requested setup.bash" >&2
  return "$_g1_source_status" 2>/dev/null || exit "$_g1_source_status"
fi
export G1_UPSTREAM_BRANCH="$( [[ "$_g1_requested" == jazzy ]] && echo jazzy || echo main )"
export G1_WS="${G1_WS:-$HOME/g1_ws}"
if [[ -f "$G1_WS/install/setup.bash" ]]; then
  _g1_had_nounset=0
  case $- in *u*) _g1_had_nounset=1; set +u ;; esac
  # shellcheck disable=SC1090
  source "$G1_WS/install/setup.bash"
  _g1_source_status=$?
  if [[ $_g1_had_nounset -eq 1 ]]; then set -u; fi
  if [[ $_g1_source_status -ne 0 ]]; then
    echo "Unable to source $G1_WS/install/setup.bash" >&2
    return "$_g1_source_status" 2>/dev/null || exit "$_g1_source_status"
  fi
fi
unset _g1_had_nounset _g1_source_status
echo "Selected ROS 2 $ROS_DISTRO; DFKI branch $G1_UPSTREAM_BRANCH; workspace $G1_WS"
unset _g1_requested
