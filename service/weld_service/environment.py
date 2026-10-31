"""Read the project's simple KEY=value .env without ROS-side dependencies."""
import os
import re
import shlex
from pathlib import Path


def load_environment(path):
    """Support comments, empty values and quoted values; preserve explicit env."""
    path = Path(path)
    if not path.is_file():
        return
    for number, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, raw_value = line.removeprefix("export ").partition("=")
        key = key.strip()
        if not separator or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key) is None:
            raise ValueError(f"{path}:{number}: expected KEY=value")
        try:
            words = shlex.split(raw_value, comments=True, posix=True)
        except ValueError as exc:
            raise ValueError(f"{path}:{number}: {exc}") from exc
        if len(words) > 1:
            raise ValueError(f"{path}:{number}: quote values containing spaces")
        os.environ.setdefault(key, words[0] if words else "")
