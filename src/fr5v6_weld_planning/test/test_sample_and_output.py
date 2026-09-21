import csv
import json

import pytest
import yaml

from fr5v6_weld_planning.config import ConfigError, load_config
from fr5v6_weld_planning.mesh_loader import load_stl
from fr5v6_weld_planning.output_writer import write_outputs
from fr5v6_weld_planning.pose_generator import generate_weld_path
from fr5v6_weld_planning.sample_workpiece import create_t_coupon_stl
from fr5v6_weld_planning.seam_extractor import extract_candidates, select_candidate
from fr5v6_weld_planning.seam_processor import resample_chain


def extraction_config():
    return {
        'min_dihedral_deg': 45.0,
        'max_dihedral_deg': 135.0,
        'min_seam_length': 0.15,
        'prefer_concave': True,
    }


def test_generated_t_coupon_has_a_200mm_concave_candidate(tmp_path):
    path = tmp_path / 'workpiece.stl'
    create_t_coupon_stl(path)

    candidates = extract_candidates(load_stl(path), extraction_config())

    assert candidates
    assert candidates[0].concave is True
    assert candidates[0].length == pytest.approx(0.2)


def test_config_rejects_non_metre_units(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump({'units': {'length': 'mm'}}), encoding='utf-8')

    with pytest.raises(ConfigError, match='metres'):
        load_config(path)


def test_output_files_have_stable_schema(tmp_path):
    stl = tmp_path / 'workpiece.stl'
    create_t_coupon_stl(stl)
    candidates = extract_candidates(load_stl(stl), extraction_config())
    selected = select_candidate(candidates)
    samples = resample_chain(selected, 0.05)
    path = generate_weld_path(samples, {
        'sample_spacing': 0.05, 'approach_distance': 0.05, 'retreat_distance': 0.05,
    })

    write_outputs(tmp_path, candidates, selected, path, {'frame_id': 'workpiece_frame'})

    detected = json.loads((tmp_path / 'detected_seams.json').read_text())
    assert detected['selected'] == selected.name
    with (tmp_path / 'weld_path.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0].keys() == {
        'index', 'segment', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
        'tx', 'ty', 'tz', 'n1x', 'n1y', 'n1z', 'n2x', 'n2y', 'n2z',
    }
    assert len(rows) == len(path)
    data = yaml.safe_load((tmp_path / 'weld_path.yaml').read_text())
    assert data['frame_id'] == 'workpiece_frame'
    assert len(data['points']) == len(path)
