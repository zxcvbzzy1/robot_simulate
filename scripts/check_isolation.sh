#!/usr/bin/env bash
set -euo pipefail
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for package in fr5v6_moveit_config fr5v6_ros2_control fr5v6_weld_interfaces fr5v6_weld_planning; do
  prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)" || {
    printf '错误：找不到 %s；请先运行 ./scripts/build.sh。\n' "${package}" >&2
    exit 69
  }
  if [[ "${prefix}" != "${ROOT}/install/"* ]]; then
    printf '错误：%s 来自错误工作区：%s\n' "${package}" "${prefix}" >&2
    exit 70
  fi
done
printf '隔离检查通过：ROS_DOMAIN_ID=%s GZ_PARTITION=%s\n' "${ROS_DOMAIN_ID}" "${GZ_PARTITION}"
