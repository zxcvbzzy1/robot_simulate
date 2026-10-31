"""Stateless service operations and task identity checks for the Python pipeline.

Only these adapters know ROS messages. Geometry modules remain usable offline.
Config paths are absolute paths on this simulator's shared local filesystem.
"""
import hashlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from fr5v6_weld_interfaces.srv import DetectSeams, GeneratePaths

from .config import load_config
from .mesh_loader import load_stl
from .seam_extractor import extract_candidates
from .workflow import PreparedGeometry, prepare_candidates, selected_candidates
from .ros_codec import candidate_from_msg, candidate_to_msg, path_from_msg, path_to_msg


def job_identity(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def context(config_file, job_id, expected_mesh=None):
    path = Path(config_file)
    if not path.is_absolute() or job_identity(path) != job_id:
        raise ValueError('job config identity mismatch (absolute immutable config required)')
    config = load_config(path)
    mesh = Path(config['workpiece']['mesh'])
    if not mesh.is_absolute():
        mesh = Path(get_package_share_directory('fr5v6_weld_planning')) / mesh
    checksum = job_identity(mesh)
    pinned = config.get('job', {}).get('mesh_sha256')
    if (pinned and checksum != pinned) or (expected_mesh and checksum != expected_mesh):
        raise ValueError('job mesh checksum mismatch')
    return config, mesh, checksum


def detect(config_file, job_id):
    config, mesh_path, checksum = context(config_file, job_id)
    mesh = load_stl(mesh_path, config['workpiece']['mesh_scale'])
    candidates = extract_candidates(mesh, config['extraction'])
    # Check again after reading the assets; never return mixed-version results.
    context(config_file, job_id, checksum)
    return DetectSeams.Response(success=True, job_id=job_id, mesh_sha256=checksum,
                                candidates=[candidate_to_msg(c) for c in candidates])


def decode_candidates(messages):
    candidates = [candidate_from_msg(c) for c in messages]
    if len({c.name for c in candidates}) != len(candidates):
        raise ValueError('duplicate candidate names')
    return candidates


def generate(config_file, job_id, mesh_sha256, messages):
    config, _, checksum = context(config_file, job_id, mesh_sha256)
    if not mesh_sha256:
        raise ValueError('missing mesh checksum')
    candidates = decode_candidates(messages)
    # Generation needs only the detected chains, not a second STL extraction.
    prepared = prepare_candidates(None, candidates, config)
    context(config_file, job_id, checksum)
    return GeneratePaths.Response(success=True, job_id=job_id, mesh_sha256=checksum,
                                   paths=[path_to_msg(p) for p in prepared])


def unpack(config_file, job_id, detection, generation):
    if detection.job_id != job_id or generation.job_id != job_id:
        raise ValueError('service response job identity mismatch')
    if not detection.success or not generation.success:
        raise ValueError('unsuccessful algorithm response')
    if not detection.mesh_sha256 or generation.mesh_sha256 != detection.mesh_sha256:
        raise ValueError('service response mesh checksum mismatch')
    config, mesh_path, _ = context(config_file, job_id, detection.mesh_sha256)
    candidates = decode_candidates(detection.candidates)
    selected = selected_candidates(candidates, config)
    if [p.candidate_name for p in generation.paths] != [c.name for c in selected]:
        raise ValueError('path selection or seam order mismatch')
    mesh = load_stl(mesh_path, config['workpiece']['mesh_scale'])
    result = []
    for candidate, path in zip(selected, generation.paths):
        samples, points = path_from_msg(path)
        result.append(PreparedGeometry(mesh, candidates, candidate, samples, points))
    context(config_file, job_id, detection.mesh_sha256)
    return result
