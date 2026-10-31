#!/usr/bin/env bash
set -euo pipefail
WELD_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WELD_REPOSITORY="$(dirname "$WELD_WORKSPACE")"
WELD_ENV_FILE="${WELD_ENV_FILE:-$WELD_WORKSPACE/.env}"
if [[ -f "$WELD_ENV_FILE" ]]; then
  set -a
  source "$WELD_ENV_FILE"
  set +a
fi
WELD_API_PORT="${WELD_API_PORT:-18800}"
WELD_PLANNER_PORT="${WELD_PLANNER_PORT:-18801}"
WELD_API_HOST="${WELD_API_HOST:-0.0.0.0}"
WELD_PLANNER_HOST="${WELD_PLANNER_HOST:-0.0.0.0}"
export WELD_ARTIFACT_ROOT="${WELD_ARTIFACT_ROOT:-$WELD_WORKSPACE/output/service}"
export ROBOT_SIMULATION_ROOT="${ROBOT_SIMULATION_ROOT:-$WELD_ARTIFACT_ROOT/simulations}"
export WELD_PLANNER_URL="${WELD_PLANNER_URL:-http://127.0.0.1:$WELD_PLANNER_PORT}"
export WELD_SERVICE_URL="${WELD_SERVICE_URL:-http://127.0.0.1:$WELD_API_PORT}"
export PYTHONPATH="$WELD_WORKSPACE/service${PYTHONPATH:+:$PYTHONPATH}"
export MPLBACKEND="${MPLBACKEND:-Agg}"
WELD_CONTAINER="${WELD_CONTAINER_NAME:-fr5-weld-planner}"
WELD_IMAGE="${WELD_IMAGE:-fr5-weld-planner:jazzy}"
mkdir -p "$WELD_ARTIFACT_ROOT" "$ROBOT_SIMULATION_ROOT"
case "${1:-help}" in
  install)
    conda run --no-capture-output -n test python -m pip install -r "$WELD_WORKSPACE/service/requirements.lock.txt"
    conda run --no-capture-output -n test python -m pip install --no-deps --no-build-isolation -e "$WELD_WORKSPACE/service"
    ;;
  build)
    docker build -t "$WELD_IMAGE" "$WELD_WORKSPACE/service/docker"
    ;;
  planner-start)
    WELD_MOUNTS=()
    if [[ -n "${WELD_INPUT_ROOT:-}" ]]; then
      WELD_MOUNTS=(-v "$WELD_INPUT_ROOT:$WELD_INPUT_ROOT:ro")
    fi
    docker run -d --init --name "$WELD_CONTAINER" --restart unless-stopped \
      -p "$WELD_PLANNER_HOST:$WELD_PLANNER_PORT:8801" \
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-58}" -e "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" \
      -e "WELD_WORKSPACE=$WELD_WORKSPACE" -e "WELD_ARTIFACT_ROOT=$WELD_ARTIFACT_ROOT" \
      -e "WELD_JOB_TIMEOUT_S=${WELD_JOB_TIMEOUT_S:-3600}" \
      -v "$WELD_REPOSITORY:$WELD_REPOSITORY:ro" \
      -v "$WELD_ARTIFACT_ROOT:$WELD_ARTIFACT_ROOT:rw" \
      "${WELD_MOUNTS[@]}" \
      "$WELD_IMAGE"
    ;;
  api-start)
    WELD_PYTHON="$(conda run -n test python -c 'import sys; print(sys.executable)')"
    if [[ -f "$WELD_ARTIFACT_ROOT/api.pid" ]]; then
      WELD_OLD_PID="$(cat "$WELD_ARTIFACT_ROOT/api.pid")"
      if [[ -r "/proc/$WELD_OLD_PID/cmdline" ]] && tr '\0' ' ' < "/proc/$WELD_OLD_PID/cmdline" | rg -q 'uvicorn weld_service.api:app'; then
        printf 'API PID already exists; inspect status first.\n' >&2
        exit 1
      fi
    fi
    nohup setsid "$WELD_PYTHON" -m uvicorn weld_service.api:app --host "$WELD_API_HOST" --port "$WELD_API_PORT" --workers 1 \
      > "$WELD_ARTIFACT_ROOT/api.log" 2>&1 < /dev/null &
    echo "$!" > "$WELD_ARTIFACT_ROOT/api.pid"
    sleep 1
    if ! kill -0 "$(cat "$WELD_ARTIFACT_ROOT/api.pid")" 2>/dev/null; then
      rm -f "$WELD_ARTIFACT_ROOT/api.pid"
      tail -n 20 "$WELD_ARTIFACT_ROOT/api.log" >&2
      exit 1
    fi
    printf 'API started: %s/docs\n' "$WELD_SERVICE_URL"
    ;;
  stop)
    if [[ -f "$WELD_ARTIFACT_ROOT/api.pid" ]]; then
      WELD_API_PID="$(cat "$WELD_ARTIFACT_ROOT/api.pid")"
      if [[ -r "/proc/$WELD_API_PID/cmdline" ]] && tr '\0' ' ' < "/proc/$WELD_API_PID/cmdline" | rg -q 'uvicorn weld_service.api:app'; then
        kill "$WELD_API_PID"
        for _ in {1..30}; do
          kill -0 "$WELD_API_PID" 2>/dev/null || break
          sleep 1
        done
      fi
      rm -f "$WELD_ARTIFACT_ROOT/api.pid"
    fi
    docker stop "$WELD_CONTAINER"
    docker rm "$WELD_CONTAINER"
    ;;
  status)
    curl --fail --max-time 15 "$WELD_SERVICE_URL/ready"
    ;;
  logs)
    docker logs --tail 80 "$WELD_CONTAINER"
    tail -n 80 "$WELD_ARTIFACT_ROOT/api.log"
    ;;
  test)
    conda run --no-capture-output -n test python -m pytest "$WELD_WORKSPACE/service/tests" -q
    ;;
  *)
    printf 'Usage: %s {install|build|planner-start|api-start|stop|status|logs|test}\n' "$0"
    ;;
esac
