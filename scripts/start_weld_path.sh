#!/usr/bin/env bash
set -euo pipefail
readonly WELD_PATH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${WELD_PATH_ROOT}/scripts/with_weld_env.sh" python3 -m fr5v6_weld_planning.external_cli "$@"
