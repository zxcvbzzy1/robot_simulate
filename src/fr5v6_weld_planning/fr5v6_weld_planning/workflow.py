"""Pure geometry portion of the weld workflow."""

from dataclasses import dataclass

import numpy as np

from .mesh_loader import MeshData, load_stl
from .pose_generator import WeldPathPoint, generate_weld_path
from .seam_extractor import SeamCandidate, extract_candidates, select_candidate
from .seam_processor import SampledPoint, resample_chain
from .transforms import pose_matrix, transform_pose_values
from .seam_extractor import SeamExtractionError


@dataclass(frozen=True)
class PreparedGeometry:
    mesh: MeshData
    candidates: list[SeamCandidate]
    selected: SeamCandidate
    samples: list[SampledPoint]
    path: list[WeldPathPoint]


def prepare_geometry(mesh_path, config):
    mesh = load_stl(mesh_path, config['workpiece']['mesh_scale'])
    candidates = extract_candidates(mesh, config['extraction'])
    selected = select_candidate(candidates, config['extraction'].get('candidate_index'))
    samples = resample_chain(selected, config['path']['sample_spacing'])
    path = generate_weld_path(samples, config['path'])
    return PreparedGeometry(mesh, candidates, selected, samples, path)


def prepare_job(mesh_path, config):
    mesh = load_stl(mesh_path, config['workpiece']['mesh_scale'])
    candidates = extract_candidates(mesh, config['extraction'])
    return prepare_candidates(mesh, candidates, config)


def selected_candidates(candidates, config):
    if not config.get('job', {}).get('enabled'):
        return [select_candidate(candidates, config['extraction'].get('candidate_index'))]
    selection = config['job']['selection']
    if selection == 'all_concave':
        selected = [item for item in candidates if item.concave]
    elif isinstance(selection, list) and selection and len(set(selection)) == len(selection):
        if any(type(index) is not int or index < 0 or index >= len(candidates) for index in selection):
            raise SeamExtractionError('job candidate index out of range')
        selected = [candidates[index] for index in selection]
    else:
        raise SeamExtractionError('invalid job seam selection')
    if not selected:
        raise SeamExtractionError('no concave weld candidates; inspect the mesh or select indices explicitly')
    return selected


def prepare_candidates(mesh, candidates, config):
    """Generate paths from detection results without repeating extraction."""
    result = []
    for candidate in selected_candidates(candidates, config):
        samples = resample_chain(candidate, config['path']['sample_spacing'])
        try:
            path = generate_weld_path(samples, config['path'])
        except ValueError as error:
            raise SeamExtractionError(f'{candidate.name}: {error}') from error
        result.append(PreparedGeometry(mesh, candidates, candidate, samples, path))
    return result


def world_path_poses(path, workpiece_pose):
    transform = pose_matrix(workpiece_pose['xyz'], workpiece_pose['rpy'])
    return [transform_pose_values(point.position, point.quaternion, transform) for point in path]


def process_segment_dts(path, speeds):
    result = []
    for first, second in zip(path, path[1:]):
        speed = float(speeds[second.segment])
        if speed <= 0:
            raise ValueError(f'{second.segment} speed must be positive')
        result.append(float(np.linalg.norm(second.position - first.position)) / speed)
    return result
