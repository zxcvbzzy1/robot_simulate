"""Paper-inspired local RANSAC, supported plane intersections and poses."""

from dataclasses import dataclass
import warnings

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def normalized(value):
    value = np.asarray(value, dtype=float)
    length = np.linalg.norm(value)
    if not np.isfinite(length) or length < 1e-9:
        raise ValueError("几何向量退化，无法确定方向")
    return value / length


@dataclass
class Plane:
    center: np.ndarray
    normal: np.ndarray
    points: np.ndarray

    @property
    def offset(self):
        return float(self.normal @ self.center)

    def distances(self, points):
        return np.abs((points - self.center) @ self.normal)

    def to_dict(self, index):
        return {
            "id": f"P{index}", "centroid": self.center.tolist(),
            "normal": self.normal.tolist(), "offset_mm": self.offset,
            "point_count": len(self.points),
            "rmse_mm": float(np.sqrt(np.mean(self.distances(self.points) ** 2))),
        }


def _tls(points):
    center = points.mean(axis=0)
    _, singular, vh = np.linalg.svd(points - center, full_matrices=False)
    if len(singular) < 3 or singular[1] < 1e-6:
        raise ValueError("平面点集共线或退化")
    return center, vh[-1]


def _centroid_refit(points, rng, iterations, threshold):
    """Eqs. 5--7, followed by robust total least squares for sensor noise.

    Quadratic fitting in the paper means a second fit, not a curved surface.
    The centroid is a calculation point (it need not be a measured sample).
    """
    center = points.mean(axis=0)
    near = points[np.argsort(np.linalg.norm(points - center, axis=1))[:50]]
    candidates = []
    for _ in range(iterations):
        a, b = near[rng.choice(len(near), 2, replace=False)]
        cross = np.cross(a - center, b - center)
        if np.linalg.norm(cross) > 1e-8:
            normal = normalized(cross)
            residual = np.abs((points - center) @ normal)
            candidates.append((float(np.median(residual)), normal))
    if not candidates:
        raise ValueError("质心邻域退化，无法精拟合平面")
    normal = min(candidates, key=lambda item: item[0])[1]
    residual = np.abs((points - center) @ normal)
    core = points[residual <= max(threshold, np.quantile(residual, 0.7))]
    for _ in range(5):
        center, normal = _tls(core)
        residual = np.abs((points - center) @ normal)
        median = np.median(residual)
        mad = np.median(np.abs(residual - median))
        cutoff = min(threshold, max(0.25, median + 3.5 * 1.4826 * mad))
        core = points[residual <= cutoff]
        if len(core) < 12:
            raise ValueError("精拟合平面内点不足")
    center, normal = _tls(core)
    return Plane(center, normal, core)


def fit_planes(points, count=None, *, seed=0, iterations=100, threshold=2.0, max_planes=20):
    rng = np.random.default_rng(seed)
    remaining = points.copy()
    planes = []
    minimum = max(30, int(len(points) * 0.003))
    for _ in range(count if count is not None else max_planes):
        if len(remaining) < minimum:
            break
        tree = cKDTree(remaining)
        best = None
        for _ in range(iterations):
            first = remaining[rng.integers(len(remaining))]
            neighbors = tree.query_ball_point(first, max(10.0, threshold * 5))
            if len(neighbors) < 4:
                continue
            local = remaining[neighbors]
            local = local[np.argsort(np.linalg.norm(local - first, axis=1))[:51]]
            local = local[np.linalg.norm(local - first, axis=1) > 1e-8][:50]
            if len(local) < 2:
                continue
            a, b = local[rng.choice(len(local), 2, replace=False)]
            cross = np.cross(a - first, b - first)
            if np.linalg.norm(cross) < 1e-8:
                continue
            normal = normalized(cross)
            mask = np.abs((remaining - first) @ normal) <= threshold
            score = int(mask.sum())
            if best is None or score > best[0]:
                best = (score, mask)
        if best is None or best[0] < minimum:
            break
        try:
            plane = _centroid_refit(remaining[best[1]], rng, iterations, threshold)
        except ValueError:
            break
        duplicate = next((p for p in planes if abs(p.normal @ plane.normal) > np.cos(np.deg2rad(3))
                          and p.distances(plane.center[None, :])[0] < threshold), None)
        if duplicate is not None:
            merged = _centroid_refit(np.vstack((duplicate.points, plane.points)), rng, iterations, threshold)
            duplicate.center, duplicate.normal, duplicate.points = merged.center, merged.normal, merged.points
        else:
            planes.append(plane)
        remaining = remaining[~(best[1] | (plane.distances(remaining) <= threshold))]

    if not planes:
        return []
    # Restore the boundary strips removed by sequential rough segmentation.
    distances = np.column_stack([plane.distances(points) for plane in planes])
    labels = distances.argmin(axis=1)
    refined = []
    for index, plane in enumerate(planes):
        cloud = points[(labels == index) & (distances[:, index] <= threshold)]
        if len(cloud) < minimum:
            continue
        try:
            refined.append(_centroid_refit(cloud, rng, iterations, threshold))
        except ValueError:
            continue
    refined.sort(key=lambda p: tuple(np.round(p.center, 6)))
    return refined


def point_spacing(points):
    """Robust measured spacing, ignoring duplicate samples."""
    if len(points) < 2:
        return 1.0
    sample = points[np.linspace(0, len(points) - 1, min(2000, len(points)), dtype=int)]
    distances = cKDTree(points).query(sample, k=2)[0][:, 1]
    positive = distances[distances > 1e-8]
    return float(np.median(positive)) if len(positive) else 1.0


def _intervals(values, gap):
    values = np.sort(values)
    groups = np.split(values, np.flatnonzero(np.diff(values) > gap) + 1)
    return [(float(g[0]), float(g[-1]), len(g)) for g in groups if len(g) >= 4]


def extract_segments(planes, points, *, threshold=2.0, min_length=3.0, max_gap=None):
    """Finite intersections supported by BOTH measured faces, never template edges."""
    if len(planes) < 2:
        return [], {"reason": "有效相交平面不足两个"}
    spacing = point_spacing(points)
    band = max(threshold * 1.5, spacing * 2)
    gap = max_gap if max_gap is not None else max(spacing * 3, threshold * 2)
    distances = np.column_stack([p.distances(points) for p in planes])
    labels = distances.argmin(axis=1)
    result, diagnostics = [], {"spacing_mm": spacing, "max_gap_mm": gap, "rejected_pairs": []}
    for i, a in enumerate(planes):
        for j in range(i + 1, len(planes)):
            b = planes[j]
            cross = np.cross(a.normal, b.normal)
            if np.linalg.norm(cross) < np.sin(np.deg2rad(5)):
                continue
            tangent = normalized(cross)
            if tangent[np.argmax(np.abs(tangent))] < 0:
                tangent = -tangent
            normals = np.array([a.normal, b.normal])
            center = (a.center + b.center) / 2
            origin = center + normals.T @ np.linalg.solve(normals @ normals.T,
                                                          [a.offset, b.offset] - normals @ center)
            relative = points - origin
            along = relative @ tangent
            radial = np.linalg.norm(np.cross(relative, tangent), axis=1)
            supports = [along[(labels == k) & (distances[:, k] <= threshold) & (radial <= band)] for k in (i, j)]
            intervals = [_intervals(group, gap) for group in supports]
            accepted = False
            for low_a, high_a, count_a in intervals[0]:
                for low_b, high_b, count_b in intervals[1]:
                    low, high = max(low_a, low_b), min(high_a, high_b)
                    if high - low < min_length:
                        continue
                    start, end = origin + low * tangent, origin + high * tangent
                    result.append({"start": start.tolist(), "end": end.tolist(), "length_mm": float(high - low),
                                   "travel_direction": tangent.tolist(), "support_planes": [f"P{i + 1}", f"P{j + 1}"],
                                   "quality": {"support_point_counts": [count_a, count_b], "candidate_type": "supported_plane_intersection"}})
                    accepted = True
            if not accepted:
                diagnostics["rejected_pairs"].append({"planes": [f"P{i + 1}", f"P{j + 1}"],
                                                      "reason": "双侧交线支撑不足、未接触或长度不足"})
    result.sort(key=lambda s: tuple(np.round(s["start"] + s["end"], 6)))
    for i, seam in enumerate(result, 1):
        seam["id"] = f"L{i}"
    if not result:
        diagnostics["reason"] = "未找到有双侧实测支撑的有限相交焊缝"
    return result, diagnostics


def _edge_line(points, start, end, tangent, radial, width, step):
    relative = points - start
    along = relative @ tangent
    radial_dist = relative @ radial
    # Reject unrelated portions of the plane and require measured edge support.
    lateral = relative - along[:, None] * tangent
    mask = ((along >= -step) & (along <= np.linalg.norm(end - start) + step)
            & (radial_dist > 0.25) & (radial_dist <= width)
            & (np.linalg.norm(lateral, axis=1) <= width * 1.15))
    local, along, radial_dist = points[mask], along[mask], radial_dist[mask]
    if len(local) < 12:
        raise ValueError("姿态邻域缺少有效边缘支撑点")
    probes = np.linspace(0, np.linalg.norm(end - start),
                         max(3, int(np.ceil(np.linalg.norm(end - start) / step)) + 1))
    coverage = cKDTree(along[:, None]).query(probes[:, None])[0]
    if coverage.max() > max(2 * step, 2.0):
        raise ValueError("姿态边缘支撑未覆盖整条焊缝")
    bins = np.floor(along / max(step * 2, 2.0)).astype(int)
    edge = []
    for key in np.unique(bins):
        indices = np.flatnonzero(bins == key)
        if len(indices) < 2:
            continue
        chosen = indices[radial_dist[indices] >= np.quantile(radial_dist[indices], 0.75)]
        edge.extend(local[chosen])
    if len(edge) < 6 or len(np.unique(bins)) < 3:
        raise ValueError("姿态边线的纵向支撑不足")
    edge = np.asarray(edge)
    center = edge.mean(axis=0)
    _, singular, vh = np.linalg.svd(edge - center, full_matrices=False)
    if singular[0] < 1e-6 or abs(vh[0] @ tangent) < 0.85:
        raise ValueError("姿态边线不平行于焊缝")
    # Eq. 24 needs parallel edges at equal longitudinal interpolation positions.
    # Constrain the SVD fit to the already estimated seam tangent.
    return center, len(local)


def plan_seam_pose(seam, planes, points, viewpoint, *, sample_step_mm=2.0,
                   posture_width_mm=5.0, approach_distance_mm=10.0,
                   retract_distance_mm=10.0, tcp_rotation_offset_deg=(0, 0, 0)):
    start, end = np.asarray(seam["start"]), np.asarray(seam["end"])
    tangent = normalized(end - start)
    ratio = seam["length_mm"] / sample_step_mm
    if not np.isfinite(ratio) or ratio > 49997:
        raise ValueError("单条轨迹超过 50000 点，请增大 sample_step_mm")
    samples = np.linspace(start, end, max(2, int(np.ceil(ratio)) + 1))
    pair = [planes[int(name[1:]) - 1] for name in seam["support_planes"]]
    distances = np.column_stack([p.distances(points) for p in planes])
    labels = distances.argmin(axis=1)
    radial = normalized(np.cross(tangent, pair[0].normal))
    if radial @ (pair[0].center - (start + end) / 2) < 0:
        radial = -radial
    directions = [radial]
    if pair[0] is pair[1]:
        directions.append(-radial)
    else:
        radial = normalized(np.cross(tangent, pair[1].normal))
        if radial @ (pair[1].center - (start + end) / 2) < 0:
            radial = -radial
        directions.append(radial)
    edges, support = [], []
    for plane, radial in zip(pair, directions):
        index = next(i for i, candidate in enumerate(planes) if candidate is plane)
        cloud = points[(labels == index) & (distances[:, index] <= 2.0)]
        edge, count = _edge_line(cloud, start, end, tangent, radial, posture_width_mm, sample_step_mm)
        edges.append(edge)
        support.append(count)
    if pair[0] is pair[1]:
        zs = pair[0].normal.copy()
        method = "coplanar_surface_normal"
    else:
        vectors = [edge - start - np.dot(edge - start, tangent) * tangent for edge in edges]
        zs = normalized(sum(normalized(v) for v in vectors))
        method = "dihedral_edge_bisector"
    if zs @ (np.asarray(viewpoint) - (start + end) / 2) < 0:
        zs = -zs
    zg = -normalized(zs - (zs @ tangent) * tangent)
    yg = normalized(np.cross(zg, tangent))  # Correct right-handed version of Eq. 25.
    rotation = np.column_stack((tangent, yg, zg))
    correction = Rotation.from_euler("xyz", tcp_rotation_offset_deg, degrees=True).as_matrix()
    rotation = rotation @ correction
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-7)):
        raise ValueError("焊枪姿态旋转矩阵无效")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)  # Deterministic equivalent RPY at gimbal lock.
        angles = Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)
    poses = np.column_stack((samples, np.tile(angles, (len(samples), 1))))
    approach = np.r_[start - zg * approach_distance_mm, angles].tolist()
    retract = np.r_[end - zg * retract_distance_mm, angles].tolist()
    waypoints = [{"stage": "APPROACH", "pose": approach}]
    waypoints.extend({"stage": "WELDING", "pose": p.tolist()} for p in poses)
    waypoints.append({"stage": "RETRACT", "pose": retract})
    return {"executable": True, "poses": poses.tolist(), "approach_pose": approach,
            "retract_pose": retract, "waypoints": waypoints,
            "quality": {"orientation_method": method, "edge_support_points": support,
                        "rotation_determinant": float(np.linalg.det(rotation))}}
