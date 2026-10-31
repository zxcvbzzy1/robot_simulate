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


def write_job_outputs(output_dir, prepared, metadata):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    summary = dict(metadata)
    summary['schema_version'] = 2
    summary['selection_rule'] = 'geometry candidates; not inferred welding process semantics'
    summary['selected_seams'] = [item.selected.name for item in prepared]
    summary['candidates'] = [{'name': c.name, 'concave': bool(c.concave), 'length_m': c.length,
                              'points': [_numbers(p) for p in c.points]}
                             for c in prepared[0].candidates]
    records = []
    for item in prepared:
        write_outputs(output / 'seams' / item.selected.name, item.candidates,
                      item.selected, item.path, metadata)
        records.extend(dict(seam_id=item.selected.name, **_path_record(index, point))
                       for index, point in enumerate(item.path))
    (output / 'detected_seams.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    with (output / 'weld_path.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['seam_id'] + CSV_FIELDS)
        writer.writeheader()
        writer.writerows(records)
    (output / 'weld_path.yaml').write_text(yaml.safe_dump(
        dict(summary, points=records), sort_keys=False), encoding='utf-8')
