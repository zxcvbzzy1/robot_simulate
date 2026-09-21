"""Stable JSON/CSV/YAML artifacts for seam extraction and path review."""

import csv
import json
from pathlib import Path

import yaml


CSV_FIELDS = [
    'index', 'segment', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
    'tx', 'ty', 'tz', 'n1x', 'n1y', 'n1z', 'n2x', 'n2y', 'n2z',
]


def _numbers(values):
    return [float(value) for value in values]


def _path_record(index, point):
    position = _numbers(point.position)
    quaternion = _numbers(point.quaternion)
    tangent = _numbers(point.tangent)
    first = _numbers(point.normal_1)
    second = _numbers(point.normal_2)
    values = [index, point.segment] + position + quaternion + tangent + first + second
    return dict(zip(CSV_FIELDS, values))


def write_outputs(output_dir, candidates, selected, path, metadata):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    detected = {
        'frame_id': metadata.get('frame_id', 'workpiece_frame'),
        'selected': selected.name,
        'candidates': [{
            'name': item.name,
            'length_m': float(item.length),
            'concave': bool(item.concave),
            'points': [_numbers(point) for point in item.points],
        } for item in candidates],
    }
    (output / 'detected_seams.json').write_text(
        json.dumps(detected, indent=2), encoding='utf-8')
    records = [_path_record(index, point) for index, point in enumerate(path)]
    with (output / 'weld_path.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(records)
    yaml_data = dict(metadata)
    yaml_data['points'] = records
    (output / 'weld_path.yaml').write_text(
        yaml.safe_dump(yaml_data, sort_keys=False), encoding='utf-8')
