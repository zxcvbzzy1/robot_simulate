#!/usr/bin/env bash
set -euo pipefail
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${ROOT}/scripts/with_weld_env.sh" bash --noprofile --norc -c \
  'set -euo pipefail; root="$1"; shift; "$root/scripts/check_isolation.sh"; exec ros2 launch fr5v6_weld_planning weld_demo.launch.py "$@"' \
  weld-launch "${ROOT}" "$@"
