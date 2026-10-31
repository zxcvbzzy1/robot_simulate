"""Own and clean up the Gazebo descendants of a single imported-model job."""
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def matching_processes(key, value, gazebo_only=False):
    expected = f'{key}={value}'.encode()
    result = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            environment = (entry / 'environ').read_bytes().split(b'\0')
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ')
            if expected in environment and (not gazebo_only or b'gz sim' in command):
                result.append(int(entry.name))
        except (OSError, ProcessLookupError):
            continue
    return result


def run_simulation(command):
    partition = os.environ.get('GZ_PARTITION', 'fr5v6_weld_seam')
    existing = matching_processes('GZ_PARTITION', partition, gazebo_only=True)
    if existing:
        raise RuntimeError(f'Gazebo already running in partition {partition} (PIDs {existing}); close that simulation first')
    token = uuid.uuid4().hex
    environment = dict(os.environ, FR5_WELD_JOB_OWNER=token)
    process = subprocess.Popen(command, env=environment, start_new_session=True)
    try:
        return process.wait()
    finally:
        # Gazebo creates its own process groups; killing only ros2 launch can
        # leave a server behind. Signal only descendants with this unique tag.
        for sig, seconds in ((signal.SIGINT, 4.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            owned = matching_processes('FR5_WELD_JOB_OWNER', token)
            if not owned:
                break
            for pid in owned:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if not matching_processes('FR5_WELD_JOB_OWNER', token):
                    break
                time.sleep(0.1)
        process.wait(timeout=2)
