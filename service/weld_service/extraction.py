"""STL adapter emitting exactly the point-cloud trajectory contract."""
import csv
import hashlib
import json
import shutil
from pathlib import Path
from uuid import uuid4
import numpy as np
from scipy.spatial.transform import Rotation

from .settings import ARTIFACT_ROOT, legacy_modules
from .trajectory import atomic_json, pose_matrix, matrix_pose, load_trajectory
from .point_cloud.pipeline import render_preview


def extract_stl(stl_path, unit, pose_in_base, seam_indices=None, **parameters):
    legacy_modules()
    from fr5v6_weld_planning.mesh_loader import load_stl
    from fr5v6_weld_planning.seam_extractor import extract_candidates
    from fr5v6_weld_planning.seam_processor import resample_chain
    from fr5v6_weld_planning.pose_generator import generate_weld_path

    source = Path(stl_path).expanduser().resolve()
    raw = source.read_bytes()
    # Parse the immutable input copy, rather than reading a changing source twice.
    identity = uuid4().hex
    root = ARTIFACT_ROOT / "extractions"
    root.mkdir(parents=True, exist_ok=True)
    destination = root / identity
    staging = root / f".{identity}.tmp"
    staging.mkdir()
    try:
        (staging / "input.stl").write_bytes(raw)
        mesh = load_stl(staging / "input.stl", 0.001 if unit == "mm" else 1)
        candidates = extract_candidates(mesh, {
            "min_dihedral_deg": 45., "max_dihedral_deg": 135.,
            "min_seam_length": parameters["min_seam_length_mm"] / 1000,
            "prefer_concave": True, "max_chain_turn_deg": 20.,
        })
        if seam_indices is not None and any(i >= len(candidates) for i in seam_indices):
            raise ValueError("seam_indices contains an unknown candidate index")
        selected = [candidates[i] for i in seam_indices] if seam_indices is not None else [c for c in candidates if c.concave]
        transform = pose_matrix(pose_in_base)
        seams = []
        for candidate in selected:
            samples = resample_chain(candidate, parameters["sample_step_mm"] / 1000)
            path = generate_weld_path(samples, {
                "sample_spacing": parameters["sample_step_mm"] / 1000,
                "approach_distance": parameters["approach_distance_mm"] / 1000,
                "retreat_distance": parameters["retract_distance_mm"] / 1000,
            })
            points = candidate.points @ transform[:3, :3].T + transform[:3, 3]
            waypoints = []
            for point in path:
                local = np.eye(4)
                local[:3, :3] = Rotation.from_quat(point.quaternion).as_matrix()
                local[:3, 3] = point.position
                waypoints.append({"stage": {"approach": "APPROACH", "seam": "WELDING", "retreat": "RETRACT"}[point.segment],
                                  "pose": matrix_pose(transform @ local)})
            seams.append({"id": candidate.name, "start": (points[0] * 1000).tolist(),
                          "end": (points[-1] * 1000).tolist(), "length_mm": candidate.length * 1000,
                          "executable": True, "quality": {"concave": candidate.concave},
                          "poses": [w["pose"] for w in waypoints if w["stage"] == "WELDING"], "waypoints": waypoints})
        warnings = [] if seams else ["No selected concave weld candidates; inspect geometry or select seam_indices explicitly."]
        document = {"schema_version": 1, "frame": "robot_base", "units": {"position": "mm", "angle": "degree"},
                    "rotation_convention": "Rz(rz) Ry(ry) Rx(rx)", "tcp_origin": "welding_tip",
                    "source": {"stl_path": str(source), "sha256": hashlib.sha256(raw).hexdigest(),
                               "unit": unit, "pose_in_base": pose_in_base}, "parameters": parameters,
                    "warnings": warnings, "seams": seams}
        atomic_json(staging / "trajectory.json", document)
        load_trajectory(staging / "trajectory.json")
        with (staging / "trajectory.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["seam_id", "point_index", "stage", "x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"])
            for seam in seams:
                for index, point in enumerate(seam["waypoints"]):
                    writer.writerow([seam["id"], index, point["stage"], *point["pose"]])
        preview = mesh.vertices @ transform[:3, :3].T + transform[:3, 3]
        render_seams = [{**s, "approach_pose": s["waypoints"][0]["pose"], "retract_pose": s["waypoints"][-1]["pose"]} for s in seams]
        render_preview(preview * 1000, render_seams, staging / "seams.png")
        staging.rename(destination)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    image = str(destination / "seams.png")
    return {"trajectory_path": str(destination / "trajectory.json"), "csv_path": str(destination / "trajectory.csv"),
            "image_path": image, "frame": "robot_base", "seam_count": len(seams), "executable_seam_count": len(seams),
            "point_count": sum(len(s["waypoints"]) for s in seams), "warnings": warnings,
            "intersection_quality": None, "statistics": {"candidate_count": len(candidates)},
            "seams": [{k: s[k] for k in ("id", "start", "end", "length_mm", "executable", "quality")} for s in seams],
            "inline_artifact_arguments": {"artifact_type": "image", "image": {
                "title": "STL 焊缝提取与轨迹预览", "file_path": image, "alt": "STL weld seams", "mime_type": "image/png"}}}
