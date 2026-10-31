#!/usr/bin/env bash
set -euo pipefail
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${ROOT}/scripts/with_weld_env.sh" python3 -m fr5v6_weld_planning.job_cli "$@"
