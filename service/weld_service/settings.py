"""Paths shared with the container; no ROS imports in the public API."""
import os
from pathlib import Path
from .environment import load_environment

WORKSPACE = Path(__file__).resolve().parents[2]
ENV_FILE = Path(os.getenv("WELD_ENV_FILE", str(WORKSPACE / ".env"))).expanduser()
if not ENV_FILE.is_absolute():
    ENV_FILE = WORKSPACE / ENV_FILE
load_environment(ENV_FILE)


def workspace_path(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else WORKSPACE / path).resolve()


ARTIFACT_ROOT = workspace_path(os.getenv("WELD_ARTIFACT_ROOT", str(WORKSPACE / "output/service")))
SNAPSHOT_ROOT = workspace_path(os.getenv("ROBOT_SIMULATION_ROOT", str(ARTIFACT_ROOT / "simulations")))
API_HOST = os.getenv("WELD_API_HOST", "0.0.0.0")
API_PORT = int(os.getenv("WELD_API_PORT", "18800"))
PLANNER_PORT = int(os.getenv("WELD_PLANNER_PORT", "18801"))
PLANNER_URL = os.getenv("WELD_PLANNER_URL", f"http://127.0.0.1:{PLANNER_PORT}").rstrip("/")
PLANNER_READY_TIMEOUT = float(os.getenv("WELD_PLANNER_READY_TIMEOUT_S", "15"))
BRIDGE_HOST = os.getenv("WELD_BRIDGE_HOST", "0.0.0.0")
BRIDGE_PORT = int(os.getenv("WELD_BRIDGE_PORT", "8801"))
JOB_TIMEOUT = float(os.getenv("WELD_JOB_TIMEOUT_S", "3600"))
UPLOAD_MAX_BYTES = int(os.getenv("WELD_UPLOAD_MAX_BYTES", str(512 * 1024 * 1024)))
# Keep subprocesses and standalone extraction imports on the same resolved paths.
os.environ["WELD_ARTIFACT_ROOT"] = str(ARTIFACT_ROOT)
os.environ["ROBOT_SIMULATION_ROOT"] = str(SNAPSHOT_ROOT)


def legacy_modules():
    import sys
    path = str(WORKSPACE / "src/fr5v6_weld_planning")
    if path not in sys.path:
        sys.path.insert(0, path)
