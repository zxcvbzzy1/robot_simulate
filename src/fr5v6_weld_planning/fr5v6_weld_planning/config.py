"""Configuration loading and validation."""

from pathlib import Path

import yaml


class ConfigError(ValueError):
    pass


def load_config(path):
    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f'unable to load configuration: {path}') from error
    if not isinstance(data, dict):
        raise ConfigError('configuration root must be a mapping')
    if data.get('units', {}).get('length') != 'm':
        raise ConfigError('all configured lengths must use metres')
    return data
