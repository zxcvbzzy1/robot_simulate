#!/usr/bin/env bash
set -euo pipefail
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHONPATH="${ROOT}/src/fr5v6_weld_planning" python3 -m fr5v6_weld_planning.tcp_config \
  --config "${ROOT}/src/fr5v6_weld_planning/config/weld_project.yaml" \
  --urdf "${ROOT}/src/fr5v6_ros2_control/urdf/fr5v6.urdf"
exec "${ROOT}/scripts/with_weld_env.sh" bash --noprofile --norc -c \
  'set -euo pipefail; cd "$1"; shift; colcon build --symlink-install "$@"' \
  weld-build "${ROOT}" "$@"
