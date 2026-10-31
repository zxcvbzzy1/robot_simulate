"""Version 1 trajectory validation and immutable snapshot utilities."""
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation
MAX_BYTES = 64 * 1024 * 1024
MAX_POINTS = 50000
def vector(value, size, name):
    if not isinstance(value, (list, tuple, np.ndarray)):
        raise ValueError(f"{name} 必须为 {size} 维有限数值数组")
    try:
        if np.asarray(value).dtype.kind not in "iuf" or any(isinstance(v, (bool, np.bool_)) for v in value):
            raise ValueError()
        result = np.asarray(value, dtype=float)
    except (ValueError, TypeError):
        raise ValueError(f"{name} 必须为 {size} 维有限数值数组") from None
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} 必须为 {size} 维有限数值数组")
    return result


def pose_matrix(pose):
    pose = vector(pose, 6, "pose")
    transform = np.eye(4)
    transform[:3, 3] = pose[:3] / 1000
    transform[:3, :3] = Rotation.from_euler("xyz", pose[3:], degrees=True).as_matrix()
    return transform


def matrix_pose(transform):
    return np.r_[transform[:3, 3] * 1000, Rotation.from_matrix(transform[:3, :3]).as_euler("xyz", degrees=True)].tolist()


def strict_json(raw):
    def invalid(value):
        raise ValueError(f"JSON 含非法数值 {value}")
    try:
        return json.loads(raw, parse_constant=invalid)
    except (UnicodeError, json.JSONDecodeError):
        raise ValueError("轨迹文件不是有效的 UTF-8 JSON") from None


def load_trajectory(path, seam_id=None):
    if not isinstance(path, (str, os.PathLike)) or not str(path).strip():
        raise ValueError("trajectory_path 必须为 JSON 文件路径")
    if seam_id is not None and (not isinstance(seam_id, str) or not seam_id.strip()):
        raise ValueError("seam_id 必须为非空字符串")
    source = Path(path).expanduser().resolve()
    if source.suffix.lower() != ".json" or not source.is_file():
        raise ValueError("trajectory_path 必须指向存在的 JSON 文件")
    # Read a bounded snapshot once, so source edits cannot change the simulation.
    with source.open("rb") as stream:
        raw = stream.read(MAX_BYTES + 1)
    if len(raw) > MAX_BYTES:
        raise ValueError("轨迹 JSON 超过 64 MiB")
    data = strict_json(raw)
    if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ValueError("轨迹必须为 schema_version=1")
    if (data.get("frame") != "robot_base" or data.get("units") != {"position": "mm", "angle": "degree"}
            or data.get("rotation_convention") != "Rz(rz) Ry(ry) Rx(rx)"):
        raise ValueError("轨迹必须使用基座坐标、mm/degree 和 Rz(rz) Ry(ry) Rx(rx)")
    if not isinstance(data.get("seams"), list):
        raise ValueError("轨迹必须包含 seams 数组")
    normalized, identities, total = [], set(), 0
    for seam in data["seams"]:
        if not isinstance(seam, dict) or not isinstance(seam.get("id"), str) or not seam["id"].strip():
            raise ValueError("焊缝必须包含非空字符串 id")
        if seam["id"] in identities:
            raise ValueError("轨迹焊缝 ID 重复")
        identities.add(seam["id"])
        start = vector(seam.get("start"), 3, "seam.start").tolist()
        end = vector(seam.get("end"), 3, "seam.end").tolist()
        rows = seam.get("waypoints", [])
        if not isinstance(rows, list):
            raise ValueError("waypoints 必须为数组")
        if "poses" in seam and not isinstance(seam["poses"], list):
            raise ValueError("poses 必须为数组")
        if not rows and seam.get("poses"):
            if not isinstance(seam["poses"], list):
                raise ValueError("poses 必须为数组")
            rows = [{"stage": "WELDING", "pose": p} for p in seam["poses"]]
        total += len(rows)
        if total > MAX_POINTS:
            raise ValueError("轨迹总点数超过 50,000，不允许截断")
        points = []
        for row in rows:
            if (not isinstance(row, dict) or not isinstance(row.get("stage"), str)
                    or row["stage"] not in {"APPROACH", "WELDING", "RETRACT"}):
                raise ValueError("轨迹点 stage 必须为 APPROACH/WELDING/RETRACT")
            points.append({"stage": row["stage"], "pose": vector(row.get("pose"), 6, "waypoint.pose").tolist()})
        normalized.append({"id": seam["id"], "start": start, "end": end, "points": points})
    if seam_id is not None and seam_id not in identities:
        raise ValueError(f"焊缝不存在: {seam_id}")
    return source, raw, data, [s for s in normalized if seam_id is None or s["id"] == seam_id]


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(data, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def atomic_bytes(path, raw):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
