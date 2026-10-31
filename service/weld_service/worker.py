"""Killable extraction / HTTP planning job subprocess."""
import json
import sys
from pathlib import Path
import httpx
from .settings import PLANNER_URL, JOB_TIMEOUT, SNAPSHOT_ROOT
from .trajectory import atomic_json, load_trajectory


def execute(spec):
    args = spec["arguments"]
    if spec["kind"] == "point-cloud":
        from .point_cloud.pipeline import extract_weld_seams
        result = extract_weld_seams(**args)
        load_trajectory(result["trajectory_path"])
        return result
    if spec["kind"] == "stl":
        from .extraction import extract_stl
        return extract_stl(**args)
    with httpx.Client(timeout=JOB_TIMEOUT, trust_env=False) as client:
        response = client.post(PLANNER_URL + "/plan", json=spec)
        response.raise_for_status()
    data = response.json()
    atomic_json(SNAPSHOT_ROOT / f"{data['simulation_id']}.json", data)
    return {"type": "robot_simulation", "title": "机械臂轨迹仿真", "simulation_id": data["simulation_id"],
            "url": f"/api/im/robot-simulations/{data['simulation_id']}", "run_id": data["run_id"],
            "summary": data["summary"], "tcp_calibrated": data["tcp_calibrated"],
            "editable": False, "mime_type": "application/json"}


if __name__ == "__main__":
    try:
        spec = json.loads(Path(sys.argv[1]).read_text())
        result = execute(spec)
        atomic_json(Path(sys.argv[1]).with_name("result.json"), result)
    except Exception as exc:
        atomic_json(Path(sys.argv[1]).with_name("error.json"), {"code": type(exc).__name__, "message": str(exc)})
        raise
