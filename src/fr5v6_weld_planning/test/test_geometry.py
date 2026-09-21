import numpy as np
import pytest

from fr5v6_weld_planning.mesh_loader import MeshData
from fr5v6_weld_planning.pose_generator import generate_weld_path
from fr5v6_weld_planning.seam_extractor import (
    SeamExtractionError,
    extract_candidates,
    select_candidate,
)
from fr5v6_weld_planning.seam_processor import resample_chain


def folded_strip_mesh():
    # Two consistently wound faces meet at the x-axis with a 90 degree fold.
    vertices = np.asarray([
        [0, 0, 0], [0.2, 0, 0], [0, 0.1, 0], [0.2, 0.1, 0],
        [0, 0, 0.1], [0.2, 0, 0.1],
    ], dtype=float)
    triangles = np.asarray([[0, 1, 3], [0, 3, 2], [0, 5, 1], [0, 4, 5]])
    normals = np.cross(
        vertices[triangles[:, 1]] - vertices[triangles[:, 0]],
        vertices[triangles[:, 2]] - vertices[triangles[:, 0]],
    )
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return MeshData(vertices, triangles, normals)


def test_extracts_and_orders_long_fold_edge():
    candidates = extract_candidates(folded_strip_mesh(), {
        'min_dihedral_deg': 45.0,
        'max_dihedral_deg': 135.0,
        'min_seam_length': 0.15,
        'prefer_concave': False,
    })

    selected = select_candidate(candidates, None)
    assert selected.length == pytest.approx(0.2)
    np.testing.assert_allclose(selected.points[[0, -1]], [[0, 0, 0], [0.2, 0, 0]])
    assert selected.normals_1.shape == (2, 3)


def test_manual_candidate_index_is_validated():
    candidates = extract_candidates(folded_strip_mesh(), {
        'min_dihedral_deg': 45.0, 'max_dihedral_deg': 135.0,
        'min_seam_length': 0.15, 'prefer_concave': False,
    })
    with pytest.raises(SeamExtractionError, match='candidate_index'):
        select_candidate(candidates, 4)


def test_resampling_keeps_endpoints_and_computes_tangents():
    candidate = select_candidate(extract_candidates(folded_strip_mesh(), {
        'min_dihedral_deg': 45.0, 'max_dihedral_deg': 135.0,
        'min_seam_length': 0.15, 'prefer_concave': False,
    }), None)

    samples = resample_chain(candidate, 0.05)

    assert len(samples) == 5
    np.testing.assert_allclose(samples[0].position, [0, 0, 0])
    np.testing.assert_allclose(samples[-1].position, [0.2, 0, 0])
    for sample in samples:
        np.testing.assert_allclose(sample.tangent, [1, 0, 0])


def test_pose_frames_and_quaternions_are_continuous():
    candidate = select_candidate(extract_candidates(folded_strip_mesh(), {
        'min_dihedral_deg': 45.0, 'max_dihedral_deg': 135.0,
        'min_seam_length': 0.15, 'prefer_concave': False,
    }), None)
    samples = resample_chain(candidate, 0.05)

    path = generate_weld_path(samples, {
        'sample_spacing': 0.05,
        'approach_distance': 0.05,
        'retreat_distance': 0.05,
    })

    assert path[0].segment == 'approach'
    assert path[-1].segment == 'retreat'
    seam = [point for point in path if point.segment == 'seam']
    assert len(seam) == 5
    for first, second in zip(path, path[1:]):
        assert np.dot(first.quaternion, second.quaternion) >= 0.0
    for point in path:
        assert np.linalg.norm(point.quaternion) == pytest.approx(1.0)
        assert np.dot(point.tangent, point.torch_z) == pytest.approx(0.0, abs=1e-12)
