#!/usr/bin/env bash
set -euo pipefail

readonly WELD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly SOURCE_A="/home/gsx/FR_Planning_Benchmark_ws"
readonly SOURCE_B="/home/gsx/FR_Gazebo_Jazzy_ws"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${WELD_ROOT}/install/setup.bash" ]]; then
  source "${WELD_ROOT}/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID=58
export ROS_LOCALHOST_ONLY=1
export GZ_PARTITION=fr5v6_weld_seam
export LIBGL_DRI3_DISABLE=1
export QT_QPA_PLATFORM=xcb
export MPLCONFIGDIR="${WELD_ROOT}/.cache/matplotlib"
mkdir -p "${MPLCONFIGDIR}"
for variable in AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do
  if [[ "${!variable:-}" == *"${SOURCE_A}"* || "${!variable:-}" == *"${SOURCE_B}"* ]]; then
    printf '错误：隔离环境中的 %s 包含只读来源工作区。\n' "${variable}" >&2
    exit 70
  fi
done
if [[ $# -eq 0 ]]; then
  printf '用法: %s <命令> [参数...]\n' "$0" >&2
  exit 64
fi
exec "$@"
