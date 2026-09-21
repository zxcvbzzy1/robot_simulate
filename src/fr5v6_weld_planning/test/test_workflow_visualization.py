from pathlib import Path

import pytest

from fr5v6_weld_planning.sample_workpiece import create_t_coupon_stl
from fr5v6_weld_planning.visualization import build_markers
from fr5v6_weld_planning.workflow import prepare_geometry, process_segment_dts, world_path_poses


def minimal_config():
    return {
        'workpiece': {'mesh_scale': 1.0},
        'extraction': {
            'min_dihedral_deg': 45.0, 'max_dihedral_deg': 135.0,
            'min_seam_length': 0.15, 'prefer_concave': True,
            'max_chain_turn_deg': 20.0, 'candidate_index': None,
        },
        'path': {'sample_spacing': 0.005, 'approach_distance': 0.05,
                 'retreat_distance': 0.05},
    }


def test_prepared_sample_has_41_seam_points(tmp_path):
    mesh_path = create_t_coupon_stl(tmp_path / 'workpiece.stl')
    prepared = prepare_geometry(mesh_path, minimal_config())
    assert len(prepared.samples) == 41
    assert len([point for point in prepared.path if point.segment == 'seam']) == 41


def test_markers_cover_required_visual_layers(tmp_path):
    mesh_path = create_t_coupon_stl(tmp_path / 'workpiece.stl')
    prepared = prepare_geometry(mesh_path, minimal_config())
    markers = build_markers(
        prepared.candidates, prepared.selected, prepared.samples, prepared.path,
        Path(mesh_path).resolve().as_uri(),
        {'xyz': [0.55, 0, 0.25], 'rpy': [0, 0, 0]},
    )
    namespaces = {marker.ns for marker in markers.markers}
    assert {'workpiece', 'candidate_seams', 'selected_seam', 'sampled_seam',
            'tcp_x', 'tcp_y', 'tcp_z', 'approach', 'retreat'} <= namespaces


def test_world_poses_and_process_speed_follow_configuration(tmp_path):
    mesh_path = create_t_coupon_stl(tmp_path / 'workpiece.stl')
    prepared = prepare_geometry(mesh_path, minimal_config())
    world = world_path_poses(prepared.path, {'xyz': [0.55, 0, 0.25], 'rpy': [0, 0, 0]})
    local_first = prepared.path[0].position
    assert world[0][0].tolist() == [local_first[0] + 0.55, local_first[1], local_first[2] + 0.25]
    dts = process_segment_dts(prepared.path, {
        'approach': 0.03, 'seam': 0.01, 'retreat': 0.03,
    })
    seam_index = next(index for index, point in enumerate(prepared.path[1:], 1)
                      if point.segment == 'seam' and prepared.path[index - 1].segment == 'seam')
    assert dts[seam_index - 1] == pytest.approx(0.5)
