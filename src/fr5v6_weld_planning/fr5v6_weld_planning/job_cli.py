"""User-facing external STL job entry point."""
import argparse
from datetime import datetime
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .config import load_config
from .job_input import make_job_config, snapshot_job
from .output_writer import write_job_outputs
from .workflow import prepare_job
from .simulation_session import run_simulation


def main():
    parser = argparse.ArgumentParser(description='Import an STL and plan or execute all selected seams')
    parser.add_argument('--mesh', required=True, help='ASCII/Binary STL file')
    parser.add_argument('--units', choices=['m', 'mm'], required=True, help='units used by the STL vertices')
    parser.add_argument('--xyz', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'), help='workpiece origin in world, metres')
    parser.add_argument('--rpy-deg', nargs=3, type=float, default=[0, 0, 0], help='workpiece RPY, degrees')
    parser.add_argument('--seams', default='all', help='all concave candidates, or ordered indices such as 0,2,1')
    parser.add_argument('--min-length', type=float, help='minimum seam length, metres')
    parser.add_argument('--output', help='new or empty run directory')
    parser.add_argument('--extract-only', action='store_true')
    parser.add_argument('--execute', action='store_true', help='execute in Gazebo after full-job preflight')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--exit-when-done', action='store_true', help='close simulation after job finishes')
    args = parser.parse_args()
    if args.extract_only and args.execute:
        parser.error('--execute cannot be combined with --extract-only')
    share = Path(get_package_share_directory('fr5v6_weld_planning'))
    output = Path(args.output).expanduser().resolve() if args.output else Path(
        '/home/gsx/FR_Weld_Seam_Planning_ws/output/jobs') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    config_path = None
    try:
        config = make_job_config(load_config(share / 'config/weld_project.yaml'), args.mesh,
                                 args.units, args.xyz, args.rpy_deg, args.seams)
        if args.min_length is not None:
            if not 0 < args.min_length < float('inf'):
                parser.error('--min-length must be positive and finite')
            config['extraction']['min_seam_length'] = args.min_length
        config, config_path = snapshot_job(config, output)
        print(f'Job directory: {output}', flush=True)
        if args.extract_only:
            prepared = prepare_job(config['workpiece']['mesh'], config)
            write_job_outputs(output, prepared, {'frame_id': 'workpiece_frame', 'units': {'length': 'm'},
                                                'mesh_sha256': config['job']['mesh_sha256']})
            print(f'Selected {len(prepared)} seams: ' + ', '.join(item.selected.name for item in prepared), flush=True)
            (output / 'execution_result.json').write_text(json.dumps({
                'status': 'extracted_only', 'execute_requested': False,
                'total_seams': len(prepared), 'completed_seams': 0}, indent=2))
            return
        command = ['ros2', 'launch', 'fr5v6_weld_planning', 'weld_demo.launch.py',
                   f'config_file:={config_path}', f'output_dir:={output}',
                   f'execute:={str(args.execute).lower()}', f'rviz:={str(not args.no_rviz).lower()}',
                   f'hold_visualization:={str(not args.exit_when_done).lower()}']
        run_simulation(command)
        result_path = output / 'execution_result.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else {}
        expected = 'succeeded' if args.execute else 'planned_only'
        if result.get('status') != expected:
            raise RuntimeError(result.get('detail', 'job interrupted or no final result'))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print(f'Weld job failed: {error}')
        if config_path is not None and not (output / 'execution_result.json').exists():
            (output / 'execution_result.json').write_text(json.dumps(
                {'status': 'failed', 'detail': str(error), 'execute_requested': args.execute,
                 'followup_motion_suppressed': True}, indent=2))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
