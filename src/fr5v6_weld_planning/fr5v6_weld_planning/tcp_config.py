"""Synchronize the configured simulation TCP into the copied robot URDF."""

import argparse
from pathlib import Path
import re

from .config import load_config


def sync_tcp_urdf(config, urdf_path):
    path = Path(urdf_path)
    text = path.read_text(encoding='utf-8')
    joint = re.compile(
        r'(<joint\s+name="tool0_to_weld_tcp"[^>]*>\s*<origin\s+)'
        r'xyz="[^"]*"\s+rpy="[^"]*"', re.MULTILINE)
    xyz = ' '.join(str(float(value)) for value in config['tcp']['xyz'])
    rpy = ' '.join(str(float(value)) for value in config['tcp']['rpy'])
    updated, count = joint.subn(rf'\g<1>xyz="{xyz}" rpy="{rpy}"', text)
    if count != 1:
        raise ValueError('expected exactly one tool0_to_weld_tcp origin in URDF')
    if updated != text:
        path.write_text(updated, encoding='utf-8')
    return updated != text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--urdf', required=True)
    args = parser.parse_args()
    changed = sync_tcp_urdf(load_config(args.config), args.urdf)
    print('Updated weld_tcp from YAML.' if changed else 'weld_tcp already matches YAML.')


if __name__ == '__main__':
    main()
