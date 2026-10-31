"""External pose-file jobs, always using this workspace's Gazebo launch."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil

import yaml

from .external_path import load_external, sha256
from .simulation_session import run_simulation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--exit-when-done', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve() if args.output else Path('/home/gsx/FR_Weld_Seam_Planning_ws/output/external_jobs') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    if output.exists() and any(output.iterdir()):
        parser.error('--output must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    try:
        document, prepared, metadata = load_external(args.config)
        copied = output / 'input_path.csv'
        shutil.copy2(metadata['path_source'], copied)
        if sha256(copied) != metadata['path_sha256']:
            raise ValueError('CSV changed while snapshotting')
        original = Path(args.config).resolve()
        shutil.copy2(original, output / 'source_job.yaml')
        if sha256(output / 'source_job.yaml') != metadata['config_sha256']:
            raise ValueError('YAML changed while snapshotting')
        document['path_file'] = 'input_path.csv'
        if document.get('workpiece'):
            mesh = (original.parent / document['workpiece']['mesh']).resolve()
            checksum = sha256(mesh)
            shutil.copy2(mesh, output / 'input.stl')
            if sha256(output / 'input.stl') != checksum:
                raise ValueError('STL changed while snapshotting')
            document['workpiece']['mesh'] = 'input.stl'
        config = output / 'job.yaml'
        config.write_text(yaml.safe_dump(document, sort_keys=False))
        print(f'Job directory: {output}; {len(prepared)} seams; {metadata["input_point_count"]} input points', flush=True)
        run_simulation(['ros2', 'launch', 'fr5v6_weld_planning', 'external_path.launch.py',
                        f'job_file:={config}', f'output_dir:={output}', f'execute:={str(args.execute).lower()}',
                        f'rviz:={str(not args.no_rviz).lower()}', f'hold_visualization:={str(not args.exit_when_done).lower()}'])
        result = json.loads((output / 'execution_result.json').read_text())
        if result.get('status') != ('succeeded' if args.execute else 'planned_only'):
            raise RuntimeError(result.get('detail', 'external job interrupted'))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        if not (output / 'execution_result.json').exists():
            (output / 'execution_result.json').write_text(json.dumps(dict(status='failed', detail=str(error),
                execute_requested=args.execute, followup_motion_suppressed=True), indent=2))
        print(f'External path failed: {error}', flush=True)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
