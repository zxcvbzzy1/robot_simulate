"""Point-cloud readers and coordinate validation; no hardware or event imports."""

from pathlib import Path

import numpy as np


def vector(value, size: int, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是 {size} 维有限数值数组") from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} 必须是 {size} 维有限数值数组")
    return result


def rigid_transform(value, name="camera_to_base") -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} 必须是有限的 4×4 刚性变换")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError(f"{name} 必须是右手刚性变换，不允许缩放、反射或透视")
    return matrix


def positive(value, name: str, *, allow_zero=False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是有限数值") from exc
    if not np.isfinite(number) or (number < 0 if allow_zero else number <= 0):
        raise ValueError(f"{name} 必须是有限的{'非负' if allow_zero else '正'}数")
    return number


def read_point_cloud(path: str) -> np.ndarray:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ValueError(f"点云文件不存在: {source}")
    try:
        if source.suffix.lower() == ".ply":
            import trimesh

            cloud = trimesh.load(source, process=False)
            if not hasattr(cloud, "vertices"):
                raise ValueError("PLY 必须包含 vertex 点集")
            points = np.asarray(cloud.vertices, dtype=float)
        elif source.suffix.lower() == ".pcd":
            points = _read_pcd(source)
        elif source.suffix.lower() == ".xyz":
            points = np.loadtxt(source, usecols=(0, 1, 2), ndmin=2)
        else:
            raise ValueError("点云仅支持 PLY、PCD、XYZ 文件")
    except (OSError, TypeError, IndexError, KeyError, UnicodeError) as exc:
        raise ValueError(f"点云文件损坏或无法读取: {exc}") from exc
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("点云至少需要三个 XYZ 点")
    return points


def _read_pcd(source: Path) -> np.ndarray:
    header = {}
    with source.open("rb") as stream:
        for _ in range(100):
            line = stream.readline()
            if not line:
                raise ValueError("PCD 缺少 DATA 头")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        else:
            raise ValueError("PCD 头过长")
        try:
            fields = header["FIELDS"]
            sizes = list(map(int, header["SIZE"]))
            types = header["TYPE"]
            counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
            count = int(header["POINTS"][0]) if "POINTS" in header else (
                int(header["WIDTH"][0]) * int(header.get("HEIGHT", ["1"])[0]))
            mode = header["DATA"][0].lower()
        except (KeyError, IndexError, ValueError) as exc:
            raise ValueError("PCD 头无效") from exc
        if (not len(fields) == len(sizes) == len(types) == len(counts)
                or len(set(fields)) != len(fields) or count < 3
                or any(c < 1 for c in counts)):
            raise ValueError("PCD 字段或点数量无效")
        if not all(axis in fields and counts[fields.index(axis)] == 1 for axis in "xyz"):
            raise ValueError("PCD 必须包含标量 x、y、z 字段")
        if mode == "binary_compressed":
            raise ValueError("暂不支持压缩 PCD，请转换为 ASCII 或普通 binary PCD")
        if mode == "ascii":
            rows = np.loadtxt(stream, ndmin=2)
            if rows.shape != (count, sum(counts)):
                raise ValueError("PCD 数据长度与头部不一致")
            offsets = np.cumsum([0, *counts[:-1]])
            return rows[:, [offsets[fields.index(axis)] for axis in "xyz"]]
        if mode != "binary":
            raise ValueError(f"不支持的 PCD DATA: {mode}")
        dtype_fields = []
        for field, kind, size, repeat in zip(fields, types, sizes, counts):
            if kind not in {"F", "I", "U"} or size not in {1, 2, 4, 8} or (kind == "F" and size not in {4, 8}):
                raise ValueError("PCD 字段类型不支持")
            dtype_fields.append((field, f"<{kind.lower()}{size}", (repeat,)))
        dtype = np.dtype(dtype_fields)
        raw = stream.read()
        if len(raw) != count * dtype.itemsize:
            raise ValueError("PCD binary 数据长度与头部不一致")
        rows = np.frombuffer(raw, dtype=dtype, count=count)
        return np.column_stack([rows[axis][:, 0] for axis in "xyz"]).astype(float)


def prepare_cloud(points, unit, input_frame, camera_to_base, camera_position_base, voxel_size_mm):
    if unit not in {"mm", "m"}:
        raise ValueError("unit 只支持 mm 或 m")
    points = np.asarray(points, dtype=float)
    original_count = len(points)
    points = points[np.isfinite(points).all(axis=1)]
    points = np.unique(points, axis=0)
    points = points * (1000.0 if unit == "m" else 1.0)
    if input_frame == "camera":
        if camera_to_base is None:
            raise ValueError("相机坐标点云必须提供 camera_to_base")
        if camera_position_base is not None:
            raise ValueError("相机坐标的相机位置由 camera_to_base 确定，无需 camera_position_base")
        transform = rigid_transform(camera_to_base)
        points = points @ transform[:3, :3].T + transform[:3, 3]
        viewpoint = transform[:3, 3]
    elif input_frame == "robot_base":
        if camera_to_base is not None:
            raise ValueError("基座坐标点云不得再次提供 camera_to_base")
        viewpoint = None if camera_position_base is None else vector(camera_position_base, 3, "camera_position_base")
    else:
        raise ValueError("input_frame 只支持 robot_base 或 camera")
    clean_count = len(points)
    # Voxel centroids retain surface location; sorted keys make input order irrelevant.
    if voxel_size_mm > 0 and len(points):
        keys = np.floor((points - points.min(axis=0)) / voxel_size_mm).astype(np.int64)
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        sums = np.zeros((len(counts), 3))
        np.add.at(sums, inverse, points)
        sampled = sums / counts[:, None]
    else:
        sampled = points.copy()
    if len(sampled) < 90:
        raise ValueError("有效点或降采样后的点不足，无法进行多平面拟合")
    return points, sampled, viewpoint, {
        "original_points": original_count, "clean_points": clean_count,
        "fitting_points": len(sampled),
    }
