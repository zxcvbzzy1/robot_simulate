"""External TCP pose files. Camera assumptions are explicit simulation data."""
import csv
from dataclasses import dataclass, replace
import hashlib
from pathlib import Path

import numpy as np
import yaml

from .transforms import matrix_from_quaternion, pose_matrix, transform_pose_values


@dataclass(frozen=True)
class ExternalPoint:
    segment: str
    position: np.ndarray
    quaternion: np.ndarray


@dataclass(frozen=True)
class ExternalSeam:
    name: str
    length: float


@dataclass(frozen=True)
class ExternalPrepared:
    selected: ExternalSeam
    path: list[ExternalPoint]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_vector(value, count, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{label}: expected {count} finite values')
    return result


def load_external(path):
    path = Path(path).expanduser().resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('unsupported external schema_version')
    if document.get('tcp_frame') != 'weld_tcp' or document.get('orientation_format') != 'quaternion_xyzw':
        raise ValueError('expected weld_tcp and quaternion_xyzw')
    scale = {'m': 1.0, 'mm': 0.001}.get(document.get('position_unit'))
    if scale is None:
        raise ValueError('position_unit must be m or mm')
    frame = document.get('frame_id')
    transform = np.eye(4)
    if frame == 'camera_frame':
        if document.get('simulation_only') is not True or document.get('calibration_status') != 'SIMULATION_ASSUMPTION':
            raise ValueError('camera_frame requires explicit SIMULATION_ASSUMPTION and simulation_only: true')
        setting = document.get('camera_to_world', {})
        transform = pose_matrix(finite_vector(setting.get('xyz'), 3, 'camera_to_world.xyz (m)'),
                                np.radians(finite_vector(setting.get('rpy_deg'), 3, 'camera_to_world.rpy_deg')))
    elif frame == 'workpiece_frame':
        setting = document.get('workpiece', {})
        transform = pose_matrix(finite_vector(setting.get('xyz'), 3, 'workpiece.xyz'),
                                np.radians(finite_vector(setting.get('rpy_deg'), 3, 'workpiece.rpy_deg')))
    elif frame != 'world':
        raise ValueError(f'unsupported frame_id: {frame}')
    if document.get('scene_mode') == 'robot_only':
        if document.get('simulation_only') is not True:
            raise ValueError('robot_only scene requires simulation_only: true')
    elif not document.get('workpiece'):
        raise ValueError('workpiece missing; explicitly select simulation-only robot_only scene')
    source = (path.parent / document['path_file']).resolve()
    before = {'config_sha256': sha256(path), 'path_sha256': sha256(source)}
    groups = {}
    with source.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        required = {'seam_id', 'index', 'segment', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV missing required fields')
        current = None
        for line, row in enumerate(reader, 2):
            name = row['seam_id'].strip()
            if not name or (name in groups and name != current):
                raise ValueError(f'CSV row {line}: empty or interleaved seam_id')
            points = groups.setdefault(name, [])
            if int(row['index']) != len(points):
                raise ValueError(f'CSV row {line}: index must be contiguous from 0')
            segment = row['segment']
            if segment not in ('approach', 'seam', 'retreat'):
                raise ValueError(f'CSV row {line}: invalid segment')
            position = finite_vector([row[k] for k in ('x', 'y', 'z')], 3, f'row {line} position') * scale
            quaternion = finite_vector([row[k] for k in ('qx', 'qy', 'qz', 'qw')], 4, f'row {line} quaternion')
            norm = np.linalg.norm(quaternion)
            if abs(norm - 1) > 1e-3:
                raise ValueError(f'CSV row {line}: non-unit quaternion')
            position, quaternion = transform_pose_values(position, quaternion / norm, transform)
            if points and np.dot(points[-1].quaternion, quaternion) < 0:
                quaternion = -quaternion
            points.append(ExternalPoint(segment, position, quaternion))
            current = name
    if not groups:
        raise ValueError('CSV contains no paths')
    settings = document.get('path', {})
    automatic = settings.get('generate_approach_retreat', True)
    if type(automatic) is not bool:
        raise ValueError('generate_approach_retreat must be boolean')
    spacing = float(settings.get('sample_spacing_m', 0.005))
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError('sample_spacing_m must be positive and finite')
    prepared = []
    for name, points in groups.items():
        seam = [p for p in points if p.segment == 'seam']
        if len(seam) < 2:
            raise ValueError(f'{name}: at least two seam points required')
        if automatic:
            if len(seam) != len(points):
                raise ValueError(f'{name}: automatic approach requires seam-only CSV')
            approach = _offset(seam[0], settings.get('approach_distance_m', 0.05), spacing, True)
            retreat = _offset(seam[-1], settings.get('retreat_distance_m', 0.05), spacing, False)
            points = approach + seam + retreat
        else:
            labels = [p.segment for p in points]
            order = {'approach': 0, 'seam': 1, 'retreat': 2}
            if set(labels) != set(order) or any(order[a] > order[b] for a, b in zip(labels, labels[1:])):
                raise ValueError(f'{name}: require ordered approach/seam/retreat')
        delta = np.linalg.norm(np.diff([p.position for p in points], axis=0), axis=1)
        if np.any(delta <= 1e-9) or np.any(delta > 0.01 + 1e-9):
            raise ValueError(f'{name}: duplicate or >10 mm path step; resample explicitly')
        length = float(np.linalg.norm(np.diff([p.position for p in seam], axis=0), axis=1).sum())
        prepared.append(ExternalPrepared(ExternalSeam(name, length), points))
    if before != {'config_sha256': sha256(path), 'path_sha256': sha256(source)}:
        raise ValueError('external input changed during import')
    metadata = dict(before, source_frame=frame, normalized_frame='world', position_unit='m',
                    path_source=str(source), input_point_count=sum(len(p) for p in groups.values()),
                    camera_to_world=document.get('camera_to_world'),
                    calibration_status=document.get('calibration_status', 'not_specified'),
                    simulation_only=bool(document.get('simulation_only')), source='external_csv',
                    workpiece_collision_geometry_present=bool(document.get('workpiece')))
    return document, prepared, metadata


def _offset(anchor, distance, spacing, approach):
    distance = float(distance)
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError('approach/retreat distance must be positive and finite')
    count = max(1, int(np.ceil(distance / spacing)))
    values = np.linspace(distance, 0, count + 1)[:-1] if approach else np.linspace(0, distance, count + 1)[1:]
    direction = -matrix_from_quaternion(anchor.quaternion)[:, 2]
    return [replace(anchor, segment='approach' if approach else 'retreat',
                    position=anchor.position + direction * value) for value in values]


def apply_settings(config, document):
    """Use world-normalized paths with the existing batch executor."""
    import copy
    result = copy.deepcopy(config)
    result['workpiece']['pose_in_world'] = {'xyz': [0., 0., 0.], 'rpy': [0., 0., 0.]}
    speed = float(document.get('path', {}).get('seam_speed_mps', 0.01))
    if not np.isfinite(speed) or not 0 < speed <= 0.03:
        raise ValueError('simulation seam speed must be positive and <=0.03 m/s')
    result['motion']['speed_mps']['seam'] = speed
    return result
