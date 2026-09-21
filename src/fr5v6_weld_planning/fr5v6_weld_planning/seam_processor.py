"""Arc-length resampling of ordered seam candidates."""

from dataclasses import dataclass

import numpy as np

from .seam_extractor import SeamCandidate, SeamExtractionError


@dataclass(frozen=True)
class SampledPoint:
    position: np.ndarray
    tangent: np.ndarray
    normal_1: np.ndarray
    normal_2: np.ndarray


def _unit(value, label):
    magnitude = float(np.linalg.norm(value))
    if magnitude <= 1e-12:
        raise SeamExtractionError(f'zero {label}')
    return value / magnitude


def resample_chain(candidate: SeamCandidate, spacing: float) -> list[SampledPoint]:
    spacing = float(spacing)
    if not np.isfinite(spacing) or spacing <= 0:
        raise SeamExtractionError('sample spacing must be positive and finite')
    points = candidate.points
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(distances <= 1e-12):
        raise SeamExtractionError('candidate contains duplicate points')
    cumulative = np.r_[0.0, np.cumsum(distances)]
    total = float(cumulative[-1])
    count = max(1, int(np.ceil(total / spacing)))
    targets = np.linspace(0.0, total, count + 1)

    positions, normals_1, normals_2 = [], [], []
    for target in targets:
        segment = min(int(np.searchsorted(cumulative, target, side='right') - 1), len(distances) - 1)
        fraction = (target - cumulative[segment]) / distances[segment]
        positions.append(points[segment] * (1 - fraction) + points[segment + 1] * fraction)
        normals_1.append(_unit(candidate.normals_1[segment] * (1 - fraction)
                               + candidate.normals_1[segment + 1] * fraction, 'normal_1'))
        normals_2.append(_unit(candidate.normals_2[segment] * (1 - fraction)
                               + candidate.normals_2[segment + 1] * fraction, 'normal_2'))
    positions = np.asarray(positions)
    tangents = []
    for index in range(len(positions)):
        if index == 0:
            delta = positions[1] - positions[0]
        elif index == len(positions) - 1:
            delta = positions[-1] - positions[-2]
        else:
            delta = positions[index + 1] - positions[index - 1]
        tangents.append(_unit(delta, 'tangent'))
    return [SampledPoint(position, tangent, first, second) for position, tangent, first, second
            in zip(positions, tangents, normals_1, normals_2)]
