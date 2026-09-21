import numpy as np
import pytest

from fr5v6_weld_planning.mesh_loader import MeshData
from fr5v6_weld_planning.scene_manager import collision_object, mesh_message, workpiece_sdf
from fr5v6_weld_planning.transforms import pose_matrix, transform_pose_values


def triangle_mesh():
    return MeshData(
        np.asarray([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float),
        np.asarray([[0, 1, 2]], dtype=int),
        np.asarray([[0, 0, 1]], dtype=float),
    )


def test_pose_matrix_uses_xyz_and_rpy_in_world():
    matrix = pose_matrix([1, 2, 3], [0, 0, np.pi / 2])
    transformed = matrix @ np.asarray([1, 0, 0, 1])
    np.testing.assert_allclose(transformed[:3], [1, 3, 3], atol=1e-12)


def test_path_pose_transform_composes_position_and_orientation():
    position, quaternion = transform_pose_values(
        [1, 2, 3], [0, 0, 0, 1],
        pose_matrix([0.5, 0, 0], [0, 0, np.pi / 2]),
    )
    np.testing.assert_allclose(position, [-1.5, 1.0, 3.0], atol=1e-12)
    assert np.linalg.norm(quaternion) == pytest.approx(1.0)


def test_moveit_mesh_preserves_indexed_geometry():
    message = mesh_message(triangle_mesh())
    assert len(message.vertices) == 3
    assert list(message.triangles[0].vertex_indices) == [0, 1, 2]


def test_gazebo_and_moveit_use_same_mesh_and_pose(tmp_path):
    mesh_path = tmp_path / 'workpiece.stl'
    mesh_path.write_text('solid placeholder\nendsolid placeholder\n')
    pose = {'xyz': [0.55, 0.0, 0.25], 'rpy': [0.0, 0.0, 0.0]}

    sdf = workpiece_sdf(mesh_path, [1, 1, 1])
    obj = collision_object('workpiece', triangle_mesh(), pose)

    assert mesh_path.resolve().as_uri() in sdf
    assert '<static>true</static>' in sdf
    assert obj.header.frame_id == 'world'
    assert obj.mesh_poses[0].position.x == pytest.approx(0.55)
    assert obj.mesh_poses[0].position.z == pytest.approx(0.25)
