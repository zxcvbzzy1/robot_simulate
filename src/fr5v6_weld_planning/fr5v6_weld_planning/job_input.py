"""Resolve a user model into a reproducible, metre-based weld job."""
import copy
import hashlib
import math
from pathlib import Path
import shutil

import yaml

from .config import ConfigError


def make_job_config(base, mesh, units, xyz, rpy_degrees, seams='all'):
    source = Path(mesh).expanduser().resolve()
    if source.suffix.lower() != '.stl' or not source.is_file():
        raise ConfigError(f'expected an existing STL file: {source}')
    if units not in ('m', 'mm'):
        raise ConfigError('mesh units must be m or mm')
    if any(len(values) != 3 or not all(math.isfinite(float(v)) for v in values)
           for values in (xyz, rpy_degrees)):
        raise ConfigError('workpiece xyz/rpy must contain three finite values')
    if seams == 'all':
        selection = 'all_concave'
    else:
        try:
            selection = [int(item.strip()) for item in seams.split(',')]
        except (ValueError, AttributeError) as error:
            raise ConfigError('seams must be all or comma-separated candidate indices') from error
        if not selection or min(selection) < 0 or len(set(selection)) != len(selection):
            raise ConfigError('candidate indices must be unique and nonnegative')
    config = copy.deepcopy(base)
    config['workpiece'].update(mesh=str(source), mesh_scale=1.0 if units == 'm' else 0.001,
                              pose_in_world={'xyz': list(map(float, xyz)),
                                             'rpy': [math.radians(v) for v in rpy_degrees]})
    config['job'] = {'enabled': True, 'selection': selection, 'source_units': units,
                     'source_mesh': str(source), 'mesh_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    return config


def snapshot_job(config, output):
    """Create a new run directory; never mix artifacts from previous jobs."""
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ConfigError(f'output directory must be empty: {output}')
    output.mkdir(parents=True, exist_ok=True)
    snapshot = output / 'input.stl'
    shutil.copy2(config['workpiece']['mesh'], snapshot)
    digest = hashlib.sha256(snapshot.read_bytes()).hexdigest()
    if digest != config['job']['mesh_sha256']:
        raise ConfigError('source mesh changed while making the job snapshot')
    config = copy.deepcopy(config)
    config['workpiece']['mesh'] = str(snapshot)
    path = output / 'job_config.yaml'
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    return config, path
