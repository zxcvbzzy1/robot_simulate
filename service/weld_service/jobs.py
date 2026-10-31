"""One durable queue and killable worker; no scientific work on the ASGI loop."""
import json
import fcntl
import os
import signal
import sqlite3
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from uuid import uuid4
import httpx
from .settings import ARTIFACT_ROOT, SNAPSHOT_ROOT, PLANNER_URL, JOB_TIMEOUT
from .trajectory import atomic_json

TERMINAL = {"succeeded", "failed", "cancelled"}


class Jobs:
    def __init__(self, root=ARTIFACT_ROOT):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.instance_lock = (self.root / "api.lock").open("a")
        try:
            fcntl.flock(self.instance_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.instance_lock.close()
            raise RuntimeError("Only one API worker may own this artifact directory") from None
        self.db = self.root / "jobs.sqlite3"
        self.lock = threading.RLock()
        self.wakeup = threading.Event()
        self.stop = threading.Event()
        self.active = None
        self.process = None
        self.blocked = False
        with self.connection() as db:
            db.execute("CREATE TABLE IF NOT EXISTS jobs (id TEXT PRIMARY KEY, kind TEXT, status TEXT, arguments TEXT, result TEXT, error TEXT, created REAL, updated REAL)")
            interrupted = db.execute("SELECT id,kind FROM jobs WHERE status='running'").fetchall()
            db.execute("UPDATE jobs SET status='failed', error=?, updated=? WHERE status IN ('queued','running')",
                       (json.dumps({"code": "interrupted", "message": "Service restarted before completion"}), time.time()))
        for identity, kind in interrupted:
            pid_file = self.root / "jobs" / identity / "process.json"
            if pid_file.is_file():
                saved = json.loads(pid_file.read_text())
                proc = Path("/proc") / str(saved["pid"])
                try:
                    if (proc / "stat").read_text().split()[21] == saved["start_time"] and "weld_service.worker" in (proc / "cmdline").read_bytes().decode():
                        os.killpg(saved["pid"], signal.SIGKILL)
                except (FileNotFoundError, ProcessLookupError):
                    pass
            if kind == "simulation":
                self.stop_remote(identity)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    @contextmanager
    def connection(self):
        db = sqlite3.connect(self.db, timeout=30)
        try:
            with db:
                yield db
        finally:
            db.close()

    def get(self, identity):
        with self.connection() as db:
            db.row_factory = sqlite3.Row
            row = db.execute("SELECT * FROM jobs WHERE id=?", (identity,)).fetchone()
        if row is None:
            raise KeyError(identity)
        data = dict(row)
        data["job_id"] = data.pop("id")
        data.pop("arguments")
        for key in ("result", "error"):
            data[key] = json.loads(data[key]) if data[key] is not None else None
        return data

    def update(self, identity, status, result=None, error=None):
        with self.connection() as db:
            db.execute("UPDATE jobs SET status=?, result=?, error=?, updated=? WHERE id=?",
                       (status, json.dumps(result) if result is not None else None,
                        json.dumps(error) if error is not None else None, time.time(), identity))

    def submit(self, kind, arguments):
        identity = uuid4().hex
        with self.connection() as db:
            db.execute("INSERT INTO jobs VALUES (?,?,?,?,NULL,NULL,?,?)",
                       (identity, kind, "queued", json.dumps(arguments), time.time(), time.time()))
        self.wakeup.set()
        return {**self.get(identity), "status_url": f"/api/v1/jobs/{identity}"}

    def stop_remote(self, identity):
        try:
            with httpx.Client(timeout=30, trust_env=False) as client:
                response = client.post(PLANNER_URL + "/cancel", json={"job_id": identity})
                response.raise_for_status()
            return True
        except httpx.HTTPError:
            # Hold the queue until the planner confirms it has no active computation.
            self.blocked = True
            return False

    def kill(self):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
            except ProcessLookupError:
                pass

    def cancel(self, identity):
        with self.lock:
            job = self.get(identity)
            if job["status"] in TERMINAL:
                return job
            if self.active == identity:
                if job["kind"] == "simulation" and not self.stop_remote(identity):
                    self.kill()
                    self.update(identity, "failed", error={"code": "cancel_unconfirmed", "message": "Planner cancellation could not be confirmed; queue paused"})
                    return self.get(identity)
                self.kill()
            self.update(identity, "cancelled")
            return self.get(identity)

    def run(self):
        while not self.stop.is_set():
            if self.blocked:
                try:
                    with httpx.Client(timeout=3, trust_env=False) as client:
                        response = client.get(PLANNER_URL + "/health")
                        response.raise_for_status()
                        self.blocked = response.json().get("busy", True)
                except httpx.HTTPError:
                    pass
                self.stop.wait(1)
                continue
            with self.lock:
                with self.connection() as db:
                    row = db.execute("SELECT id,kind,arguments FROM jobs WHERE status='queued' ORDER BY created LIMIT 1").fetchone()
                if row:
                    identity, kind, raw = row
                    self.active = identity
                    directory = self.root / "jobs" / identity
                    directory.mkdir(parents=True)
                    spec = {"job_id": identity, "kind": kind, "arguments": json.loads(raw)}
                    atomic_json(directory / "request.json", spec)
                    self.update(identity, "running")
                    env = {**os.environ, "WELD_ARTIFACT_ROOT": str(self.root), "ROBOT_SIMULATION_ROOT": str(SNAPSHOT_ROOT), "MPLBACKEND": "Agg"}
                    log = (directory / "worker.log").open("ab")
                    try:
                        self.process = subprocess.Popen([sys.executable, "-m", "weld_service.worker", str(directory / "request.json")],
                                                        env=env, stdout=log, stderr=log, start_new_session=True)
                        try:
                            start_time = Path(f"/proc/{self.process.pid}/stat").read_text().split()[21]
                            atomic_json(directory / "process.json", {"pid": self.process.pid, "start_time": start_time})
                        except FileNotFoundError:
                            pass
                    except OSError as exc:
                        log.close()
                        self.update(identity, "failed", error={"code": "worker_start", "message": str(exc)})
                        self.active = None
                        continue
            if not row:
                self.wakeup.wait(.5)
                self.wakeup.clear()
                continue
            timed_out = False
            try:
                self.process.wait(timeout=JOB_TIMEOUT)
            except subprocess.TimeoutExpired:
                timed_out = True
                with self.lock:
                    if kind == "simulation":
                        self.stop_remote(identity)
                    self.kill()
            finally:
                log.close()
            with self.lock:
                if self.get(identity)["status"] == "running":
                    result = directory / "result.json"
                    error_path = directory / "error.json"
                    if self.process.returncode == 0 and result.is_file() and not timed_out:
                        self.update(identity, "succeeded", result=json.loads(result.read_text()))
                    else:
                        error = json.loads(error_path.read_text()) if error_path.is_file() else {
                            "code": "timeout" if timed_out else "worker_failed", "message": "Job timed out" if timed_out else "Worker exited; inspect worker.log"}
                        self.update(identity, "failed", error=error)
                self.active = None
                self.process = None

    def close(self):
        self.stop.set()
        self.wakeup.set()
        if self.active:
            self.cancel(self.active)
        self.thread.join(timeout=10)
        fcntl.flock(self.instance_lock.fileno(), fcntl.LOCK_UN)
        self.instance_lock.close()
