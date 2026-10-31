"""Offline extraction entrypoint and atomic, inspectable artifacts."""

import csv
import json
from pathlib import Path
import shutil
from threading import Lock
from uuid import uuid4

import numpy as np

from .geometry import extract_segments, fit_planes, plan_seam_pose
from .io import positive, prepare_cloud, read_point_cloud, vector

from ..settings import ARTIFACT_ROOT

TEMP_DIR = ARTIFACT_ROOT / "extractions"
RENDER_LOCK = Lock()
PAPER_CORRECTIONS = [
    "Local-neighbor RANSAC and centroid refitting follow the paper's core method.",
    "Finite intersections use measured two-face support instead of workpiece templates.",
    "Eq.25: Yg=Zg cross Xg produces a right-handed rotation.",
]


def extract_weld_seams(point_cloud_path: str, model_type: str = "auto", input_frame: str = "robot_base", *,
                       unit="mm", camera_to_base=None, camera_position_base=None,
                       h_width_mm=None, tube_width_mm=None, voxel_size_mm=1.0,
                       sample_step_mm=2.0, posture_width_mm=5.0,
                       approach_distance_mm=10.0, retract_distance_mm=10.0,
                       tcp_rotation_offset_deg=(0, 0, 0), ransac_iterations=100,
                       plane_distance_threshold_mm=2.0, seed=0,
                       max_planes=20, min_seam_length_mm=3.0, max_gap_mm=None):
    if model_type not in {"auto", "three_plane", "h_section", "square_tube"}:
        raise ValueError("model_type 只支持 auto 或已弃用的旧类别")
    min_seam_length_mm = positive(min_seam_length_mm, "min_seam_length_mm")
    if min_seam_length_mm < 3:
        raise ValueError("min_seam_length_mm 不得小于 3 mm")
    notices = []
    if model_type != "auto" or h_width_mm is not None or tube_width_mm is not None:
        notices.append("model_type 和工件宽度参数已弃用，统一使用通用平面相交提取，不参与几何计算")
    parameters = {
        "min_seam_length_mm": positive(min_seam_length_mm, "min_seam_length_mm"),
        "max_gap_mm": None if max_gap_mm is None else positive(max_gap_mm, "max_gap_mm"),
        "voxel_size_mm": positive(voxel_size_mm, "voxel_size_mm", allow_zero=True),
        "sample_step_mm": positive(sample_step_mm, "sample_step_mm"),
        "posture_width_mm": positive(posture_width_mm, "posture_width_mm"),
        "approach_distance_mm": positive(approach_distance_mm, "approach_distance_mm"),
        "retract_distance_mm": positive(retract_distance_mm, "retract_distance_mm"),
        "plane_distance_threshold_mm": positive(plane_distance_threshold_mm, "plane_distance_threshold_mm"),
    }
    for name, value, upper in (("seed", seed, 2**32 - 1), ("ransac_iterations", ransac_iterations, 10000), ("max_planes", max_planes, 100)):
        if isinstance(value, bool) or not isinstance(value, int) or not (0 if name == "seed" else 1) <= value <= upper:
            raise ValueError(f"{name} 必须是范围内的整数")
        parameters[name] = value
    correction = vector(tcp_rotation_offset_deg, 3, "tcp_rotation_offset_deg")
    parameters["tcp_rotation_offset_deg"] = correction.tolist()
    for name, value in (("h_width_mm", h_width_mm), ("tube_width_mm", tube_width_mm)):
        if value is not None:
            positive(value, name)
    raw = read_point_cloud(point_cloud_path)
    points, sampled, viewpoint, statistics = prepare_cloud(
        raw, unit, input_frame, camera_to_base, camera_position_base, parameters["voxel_size_mm"])
    planes = fit_planes(sampled, seed=seed, iterations=ransac_iterations,
                        threshold=parameters["plane_distance_threshold_mm"], max_planes=max_planes)
    seams, intersection_quality = extract_segments(
        planes, points, threshold=parameters["plane_distance_threshold_mm"],
        min_length=parameters["min_seam_length_mm"], max_gap=parameters["max_gap_mm"])
    for seam in seams:
        try:
            if viewpoint is None:
                raise ValueError("未提供相机位置，仅输出焊缝几何；姿态生成需要 camera_position_base")
            geometry_quality = seam["quality"]
            seam.update(plan_seam_pose(
                seam, planes, points, viewpoint,
                **{name: parameters[name] for name in (
                    "sample_step_mm", "posture_width_mm", "approach_distance_mm",
                    "retract_distance_mm", "tcp_rotation_offset_deg")},
            ))
            seam["quality"] = {**geometry_quality, **seam["quality"]}
        except ValueError as exc:
            seam.update({"executable": False, "poses": [], "waypoints": [],
                         "quality": {**seam.get("quality", {}), "orientation_error": str(exc)}})
    document = {
        "schema_version": 1, "frame": "robot_base",
        "units": {"position": "mm", "angle": "degree"},
        "rotation_convention": "Rz(rz) Ry(ry) Rx(rx)",
        "tcp_origin": "calibrated_welding_tip", "model_type": "auto", "warnings": notices,
        "source": {"point_cloud_path": str(Path(point_cloud_path).expanduser().resolve()),
                   "input_frame": input_frame, "unit": unit,
                   "camera_position_base": None if viewpoint is None else viewpoint.tolist(),
                   "camera_to_base": None if camera_to_base is None else np.asarray(camera_to_base).tolist()},
        "parameters": parameters, "statistics": statistics,
        "planes": [plane.to_dict(i) for i, plane in enumerate(planes, 1)],
        "intersection_quality": intersection_quality,
        "paper": {"doi": "10.1016/j.rcim.2022.102433", "corrections_and_extensions": PAPER_CORRECTIONS},
        "seams": seams,
    }
    destination = TEMP_DIR / uuid4().hex
    staging = destination.with_name(f".{destination.name}.tmp")
    staging.mkdir(parents=True)
    try:
        (staging / "trajectory.json").write_text(
            json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        with (staging / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["seam_id", "point_index", "stage", "x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg"])
            for seam in seams:
                for index, waypoint in enumerate(seam["waypoints"]):
                    writer.writerow([seam["id"], index, waypoint["stage"], *waypoint["pose"]])
        render_preview(points, seams, staging / "seams.png", viewpoint=viewpoint)
        staging.rename(destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    image_path = str(destination / "seams.png")
    return {
        "trajectory_path": str(destination / "trajectory.json"), "csv_path": str(destination / "trajectory.csv"),
        "image_path": image_path, "frame": "robot_base", "seam_count": len(seams),
        "executable_seam_count": sum(seam["executable"] for seam in seams),
        "warnings": notices, "intersection_quality": intersection_quality,
        "point_count": sum(len(seam["waypoints"]) for seam in seams), "statistics": statistics,
        "seams": [{key: seam[key] for key in ("id", "start", "end", "length_mm", "executable", "quality")} for seam in seams],
        "inline_artifact_arguments": {"artifact_type": "image", "image": {
            "title": "点云焊缝提取与轨迹预览", "file_path": image_path,
            "alt": "点云、焊缝编号、运动方向与接近点", "mime_type": "image/png"}},
    }


def render_preview(points, seams, path, *, viewpoint=None):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    with RENDER_LOCK:
        figure = Figure(figsize=(10, 8), dpi=160)
        FigureCanvasAgg(figure)
        ax = figure.add_subplot(111, projection="3d")
        preview = points[np.linspace(0, len(points) - 1, min(18000, len(points)), dtype=int)]
        ax.scatter(*preview.T, s=0.8, c="#9baabc", alpha=0.25, rasterized=True)
        extents = [preview]
        colors = ["#e6550d", "#00876c", "#386cb0", "#ae017e"]
        for index, seam in enumerate(seams):
            color = colors[index % len(colors)]
            endpoints = np.asarray([seam["start"], seam["end"]])
            start, end = endpoints
            ax.plot(*endpoints.T, color=color, linewidth=3, label=seam["id"] + ("" if seam["executable"] else " (pose unavailable)"))
            ax.scatter(*endpoints.T, color=color, s=25)
            middle = (start + end) / 2
            ax.text(*middle, "  " + seam["id"], color=color, fontsize=11)
            direction = (end - start) * 0.2
            ax.quiver(*middle, *direction, color=color, arrow_length_ratio=0.35)
            extents.append(endpoints)
            if seam["executable"]:
                approach = np.asarray(seam["approach_pose"][:3])
                retract = np.asarray(seam["retract_pose"][:3])
                ax.plot(*np.vstack((approach, start)).T, "--", color=color)
                ax.plot(*np.vstack((end, retract)).T, "--", color=color)
                ax.scatter(*approach, marker="^", color=color, s=45)
                ax.text(*approach, "  A" + seam["id"][1:], fontsize=9, color=color)
                extents.append(np.vstack((approach, retract)))
        all_points = np.concatenate(extents)
        center = (all_points.min(axis=0) + all_points.max(axis=0)) / 2
        radius = max(float(np.ptp(all_points, axis=0).max()) * 0.58, 1)
        for setter, coordinate in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
            setter(coordinate - radius, coordinate + radius)
        ax.set_box_aspect((1, 1, 1))
        if viewpoint is not None:
            view = np.asarray(viewpoint) - center
            elevation = np.rad2deg(np.arctan2(view[2], np.hypot(view[0], view[1])))
            azimuth = np.rad2deg(np.arctan2(view[1], view[0]))
            ax.view_init(elev=float(np.clip(elevation, 15, 60)), azim=float(azimuth))
        ax.set(xlabel="Base X (mm)", ylabel="Base Y (mm)", zlabel="Base Z (mm)", title="Point-cloud weld seams and approach poses")
        if seams:
            ax.legend(loc="upper left")
        figure.text(0.5, 0.025, "Triangles A#: approach poses   |   Dashed lines: approach / retract", ha="center", fontsize=9, color="#455466")
        figure.subplots_adjust(left=0, right=0.96, bottom=0.08, top=0.94)
        figure.savefig(path)
        figure.clear()
