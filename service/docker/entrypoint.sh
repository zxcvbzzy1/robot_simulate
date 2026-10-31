#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="${WELD_WORKSPACE}/service:${WELD_WORKSPACE}/src/fr5v6_weld_planning:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-58}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
/usr/bin/python3 -m weld_service.headless &
launch_pid=$!
/usr/bin/python3 -m weld_service.bridge &
bridge_pid=$!
trap 'kill "$launch_pid" "$bridge_pid" 2>/dev/null || true; wait || true' EXIT INT TERM
wait -n "$launch_pid" "$bridge_pid"
