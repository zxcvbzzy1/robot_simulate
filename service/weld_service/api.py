"""Public API. Start with exactly one uvicorn worker."""
import os
import re
import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
import httpx
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from . import settings
from .jobs import Jobs
from .models import PointCloudRequest, STLRequest, SimulationRequest, JobResponse, SimulationSnapshot
from .trajectory import load_trajectory, strict_json
from .uploads import router as files_router


@asynccontextmanager
async def lifespan(app):
    logging.getLogger("uvicorn.error").info("Configuration: %s; MoveIt backend: %s", settings.ENV_FILE, settings.PLANNER_URL)
    settings.ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    settings.SNAPSHOT_ROOT.mkdir(parents=True, exist_ok=True)
    app.state.jobs = Jobs()
    try:
        yield
    finally:
        app.state.jobs.close()


app = FastAPI(title="FR5 焊缝规划服务", version="0.1.0", lifespan=lifespan,
              description="STL / 点云统一提取，MoveIt 规划及前端兼容回放。远程文件先上传，再将返回的服务端路径传给任务接口。")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
    allow_credentials=False,
)
app.include_router(files_router)


def file(path, suffixes):
    target = Path(path).expanduser().resolve()
    if not target.is_file():
        raise HTTPException(404, "Input file does not exist")
    if target.suffix.lower() not in suffixes:
        raise HTTPException(422, f"Expected file extension: {sorted(suffixes)}")
    return str(target)


async def planner_ready():
    try:
        async with httpx.AsyncClient(timeout=settings.PLANNER_READY_TIMEOUT, trust_env=False) as client:
            response = await client.get(settings.PLANNER_URL + "/ready")
            response.raise_for_status()
            data = response.json()
            if not isinstance(data, dict) or data.get("status") != "ready":
                raise ValueError("Planner did not report ready")
            return data
    except (httpx.HTTPError, ValueError) as exc:
        logging.getLogger("uvicorn.error").warning("MoveIt readiness failed at %s: %s", settings.PLANNER_URL, exc)
        raise HTTPException(503, f"MoveIt backend unavailable at {settings.PLANNER_URL}; check the container /ready endpoint and WELD_PLANNER_URL in {settings.ENV_FILE.name}") from exc


@app.get("/health", tags=["health"])
def health():
    return {"status": "ok"}


@app.get("/ready", tags=["health"])
async def ready():
    backend = await planner_ready()
    if not all(os.access(p, os.W_OK) for p in (settings.ARTIFACT_ROOT, settings.SNAPSHOT_ROOT)):
        raise HTTPException(503, "Artifact storage is not writable")
    return {"status": "ready", "planner": backend}


@app.post("/api/v1/extractions/point-cloud", status_code=202, response_model=JobResponse, tags=["extraction"])
def point_cloud(body: PointCloudRequest, request: Request):
    args = body.model_dump(exclude_none=True)
    args["point_cloud_path"] = file(body.point_cloud_path, {".ply", ".pcd", ".xyz"})
    return request.app.state.jobs.submit("point-cloud", args)


@app.post("/api/v1/extractions/stl", status_code=202, response_model=JobResponse, tags=["extraction"])
def stl(body: STLRequest, request: Request):
    args = body.model_dump()
    args["stl_path"] = file(body.stl_path, {".stl"})
    return request.app.state.jobs.submit("stl", args)


@app.post("/api/v1/simulations/", status_code=202, response_model=JobResponse, include_in_schema=False)
@app.post("/api/v1/simulations", status_code=202, response_model=JobResponse, tags=["simulation"])
async def simulation(body: SimulationRequest, request: Request):
    args = body.model_dump()
    args["trajectory_path"] = file(body.trajectory_path, {".json"})
    if body.workpiece is not None:
        args["workpiece"]["stl_path"] = file(body.workpiece.stl_path, {".stl"})
    # Validate and snapshot before queueing so later input edits cannot alter the job.
    try:
        _, raw, _, _ = await asyncio.to_thread(load_trajectory, args["trajectory_path"], body.seam_id)
    except ValueError as exc:
        raise HTTPException(422, str(exc)) from exc
    await planner_ready()
    import hashlib
    from .trajectory import atomic_bytes
    digest = hashlib.sha256(raw).hexdigest()
    snapshot = settings.ARTIFACT_ROOT / "inputs" / f"{digest}.json"
    if not snapshot.exists():
        await asyncio.to_thread(atomic_bytes, snapshot, raw)
    if body.workpiece is not None:
        mesh_path = args["workpiece"]["stl_path"]
        mesh_raw = await asyncio.to_thread(Path(mesh_path).read_bytes)
        mesh_hash = hashlib.sha256(mesh_raw).hexdigest()
        mesh_snapshot = settings.ARTIFACT_ROOT / "inputs" / f"{mesh_hash}.stl"
        await asyncio.to_thread(atomic_bytes, mesh_snapshot, mesh_raw)
        args["workpiece"].update(stl_path=str(mesh_snapshot), source_path=mesh_path, sha256=mesh_hash)
    args["source_path"] = args["trajectory_path"]
    args["source_sha256"] = digest
    args["trajectory_path"] = str(snapshot)
    return request.app.state.jobs.submit("simulation", args)


@app.get("/api/v1/jobs/{job_id}", response_model=JobResponse, tags=["jobs"])
def job(job_id: str, request: Request):
    try:
        return request.app.state.jobs.get(job_id)
    except KeyError:
        raise HTTPException(404, "Job not found") from None


@app.post("/api/v1/jobs/{job_id}/cancel", response_model=JobResponse, tags=["jobs"])
def cancel(job_id: str, request: Request):
    try:
        return request.app.state.jobs.cancel(job_id)
    except KeyError:
        raise HTTPException(404, "Job not found") from None


@app.get("/api/v1/simulations/{simulation_id}", response_model=SimulationSnapshot, tags=["simulation"])
def snapshot(simulation_id: str):
    if re.fullmatch(r"[a-f0-9]{32}", simulation_id) is None:
        raise HTTPException(404, "Simulation not found")
    try:
        return strict_json((settings.SNAPSHOT_ROOT / f"{simulation_id}.json").read_bytes())
    except FileNotFoundError:
        raise HTTPException(404, "Simulation not found") from None
