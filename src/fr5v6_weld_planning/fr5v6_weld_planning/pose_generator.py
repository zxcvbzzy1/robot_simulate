"""Generate continuous TCP frames and approach/seam/retreat segments."""

from dataclasses import dataclass
import math

import numpy as np

from .seam_extractor import SeamExtractionError


@dataclass(frozen=True)
class WeldPathPoint:
    segment: str
    position: np.ndarray
    quaternion: np.ndarray  # x, y, z, w
    tangent: np.ndarray
    normal_1: np.ndarray
    normal_2: np.ndarray
    torch_x: np.ndarray
    torch_y: np.ndarray
    torch_z: np.ndarray


def _unit(value, label):
    magnitude = float(np.linalg.norm(value))
    if magnitude <= 1e-12:
        raise SeamExtractionError(f'zero or cancelling {label}')
    return value / magnitude


def _quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return np.asarray([
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
            0.25 * s,
        ])
    diagonal = np.diag(matrix)
    index = int(np.argmax(diagonal))
    if index == 0:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
        result = [(0.25 * s), (matrix[0, 1] + matrix[1, 0]) / s,
                  (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s]
    elif index == 1:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
        result = [(matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s,
                  (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
        result = [(matrix[0, 2] + matrix[2, 0]) / s,
                  (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s,
                  (matrix[1, 0] - matrix[0, 1]) / s]
    return _unit(np.asarray(result), 'quaternion')


def _frame(sample):
    z_axis = -_unit(sample.normal_1 + sample.normal_2, 'surface-normal bisector')
    tangent = _unit(sample.tangent, 'tangent')
    x_axis = _unit(tangent - np.dot(tangent, z_axis) * z_axis, 'projected tangent')
    y_axis = _unit(np.cross(z_axis, x_axis), 'torch y-axis')
    x_axis = _unit(np.cross(y_axis, z_axis), 'torch x-axis')
    return x_axis, y_axis, z_axis, _quaternion(np.column_stack((x_axis, y_axis, z_axis)))


def generate_weld_path(samples, config) -> list[WeldPathPoint]:
    if len(samples) < 2:
        raise SeamExtractionError('at least two seam samples are required')
    spacing = float(config['sample_spacing'])
    approach = float(config['approach_distance'])
    retreat = float(config['retreat_distance'])
    if min(spacing, approach, retreat) <= 0:
        raise SeamExtractionError('path distances must be positive')
    seam = []
    previous = None
    for sample in samples:
        x_axis, y_axis, z_axis, quaternion = _frame(sample)
        if previous is not None and float(np.dot(previous, quaternion)) < 0:
            quaternion = -quaternion
        previous = quaternion
        seam.append(WeldPathPoint('seam', sample.position.copy(), quaternion, sample.tangent.copy(),
                                  sample.normal_1.copy(), sample.normal_2.copy(),
                                  x_axis, y_axis, z_axis))

    def offset_points(anchor, distance, segment, reverse):
        count = max(1, int(np.ceil(distance / spacing)))
        offsets = np.linspace(distance, 0.0, count + 1)[:-1]
        if not reverse:
            offsets = np.linspace(0.0, distance, count + 1)[1:]
        direction = -anchor.torch_z
        return [WeldPathPoint(segment, anchor.position + direction * value,
                              anchor.quaternion.copy(), anchor.tangent.copy(),
                              anchor.normal_1.copy(), anchor.normal_2.copy(),
                              anchor.torch_x.copy(), anchor.torch_y.copy(), anchor.torch_z.copy())
                for value in offsets]

    return offset_points(seam[0], approach, 'approach', True) + seam + \
        offset_points(seam[-1], retreat, 'retreat', False)
