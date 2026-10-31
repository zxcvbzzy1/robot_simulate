"""Container-local HTTP adapter with per-job process cancellation."""
import json
import os
import re
import signal
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from .settings import ARTIFACT_ROOT, BRIDGE_HOST, BRIDGE_PORT, JOB_TIMEOUT
from .trajectory import atomic_json

lock = threading.RLock()
active = None
process = None


class Handler(BaseHTTPRequestHandler):
    def send(self, status, body):
        data = json.dumps(body, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if self.path == "/health":
            with lock:
                self.send(200, {"status": "ok", "busy": active is not None, "job_id": active})
        elif self.path == "/ready":
            try:
                result = subprocess.run(["/usr/bin/python3", "-m", "weld_service.ros_worker", "--probe"], capture_output=True, timeout=10)
                self.send(200 if result.returncode == 0 else 503, {"status": "ready" if result.returncode == 0 else "unavailable", "backend": "MoveIt2"})
            except subprocess.TimeoutExpired:
                self.send(503, {"status": "unavailable"})
        else:
            self.send(404, {"error": "Not found"})

    def do_POST(self):
        global active, process
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 1024 * 1024:
                raise ValueError("Invalid request size")
            spec = json.loads(self.rfile.read(length))
            identity = spec.get("job_id", "")
            if re.fullmatch(r"[a-f0-9]{32}", identity) is None:
                raise ValueError("Invalid job_id")
        except (ValueError, TypeError, AttributeError):
            self.send(422, {"error": "Invalid JSON job request"})
            return
        if self.path == "/cancel":
            with lock:
                # A tombstone also covers cancellation before the /plan request arrives.
                directory = ARTIFACT_ROOT / "planner" / identity
                directory.mkdir(parents=True, exist_ok=True)
                (directory / "cancelled").touch()
                if active == identity and process is not None and process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                    except ProcessLookupError:
                        pass
                if active == identity:
                    active = None
                    process = None
                self.send(200, {"job_id": identity, "status": "cancelled"})
            return
        if self.path != "/plan":
            self.send(404, {"error": "Not found"})
            return
        directory = ARTIFACT_ROOT / "planner" / identity
        with lock:
            if active is not None:
                self.send(409, {"error": "Planner busy"})
                return
            if (directory / "cancelled").exists():
                self.send(409, {"error": "Job cancelled"})
                return
            directory.mkdir(parents=True, exist_ok=True)
            atomic_json(directory / "request.json", spec)
            log = (directory / "planner.log").open("ab")
            process = subprocess.Popen(["/usr/bin/python3", "-m", "weld_service.ros_worker",
                                        str(directory / "request.json"), str(directory / "result.json")],
                                       stdout=log, stderr=log, start_new_session=True)
            active = identity
            child = process
        try:
            try:
                child.wait(timeout=JOB_TIMEOUT)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
            with lock:
                if active == identity:
                    active = None
                    process = None
            if child.returncode == 0 and (directory / "result.json").exists() and not (directory / "cancelled").exists():
                self.send(200, json.loads((directory / "result.json").read_text()))
            else:
                self.send(500, {"error": "MoveIt job failed", "log_path": str(directory / "planner.log")})
        finally:
            log.close()
            with lock:
                if active == identity:
                    active = None
                    process = None


if __name__ == "__main__":
    ThreadingHTTPServer((BRIDGE_HOST, BRIDGE_PORT), Handler).serve_forever()
