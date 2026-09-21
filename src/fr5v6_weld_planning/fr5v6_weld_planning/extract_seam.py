"""Offline STL seam extraction entry point."""

import argparse
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from .config import load_config
from .output_writer import write_outputs
from .workflow import prepare_geometry


def main():
    try:
        share = Path(get_package_share_directory('fr5v6_weld_planning'))
    except PackageNotFoundError:
        share = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(share / 'config/weld_project.yaml'))
    parser.add_argument('--mesh')
    parser.add_argument('--output', default='/home/gsx/FR_Weld_Seam_Planning_ws/output')
    args = parser.parse_args()
    config = load_config(args.config)
    mesh = Path(args.mesh) if args.mesh else share / config['workpiece']['mesh']
    prepared = prepare_geometry(mesh, config)
    write_outputs(Path(args.output), prepared.candidates, prepared.selected, prepared.path,
                  {'schema_version': 1, 'frame_id': 'workpiece_frame'})
    print(f'{len(prepared.candidates)} candidates; selected {prepared.selected.name}; '
          f'{len(prepared.path)} TCP points')


if __name__ == '__main__':
    main()
