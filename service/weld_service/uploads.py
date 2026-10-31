"""Temporary input uploads and downloads of a job's registered outputs."""
import hashlib
import re
import shutil
import tempfile
from pathlib import Path
from typing import Literal
from uuid import uuid4

from fastapi import APIRouter, File, HTTPException, Request, Response, UploadFile
from fastapi.responses import FileResponse
from . import settings
from .models import UploadedFile
from .trajectory import atomic_json

router = APIRouter(prefix="/api/v1", tags=["files"])
SUFFIXES = {".ply", ".pcd", ".xyz", ".stl", ".json"}


@router.post("/files", status_code=201, response_model=UploadedFile)
def upload_file(file: UploadFile = File(...)):
    """Upload bytes first; use the returned file_path in the existing JSON API."""
    filename = (file.filename or "").replace("\\", "/").rsplit("/", 1)[-1]
    suffix = Path(filename).suffix.lower()
    if suffix not in SUFFIXES:
        raise HTTPException(422, f"Supported upload extensions: {sorted(SUFFIXES)}")
    if file.size is not None and file.size > settings.UPLOAD_MAX_BYTES:
        raise HTTPException(413, "Upload exceeds WELD_UPLOAD_MAX_BYTES")
    root = settings.ARTIFACT_ROOT / "uploads"
    root.mkdir(parents=True, exist_ok=True)
    identity = uuid4().hex
    destination = root / identity
    staging = Path(tempfile.mkdtemp(prefix=".upload-", dir=root))
    name = "input" + suffix
    size, digest = 0, hashlib.sha256()
    try:
        with (staging / name).open("wb") as stream:
            while chunk := file.file.read(1024 * 1024):
                size += len(chunk)
                if size > settings.UPLOAD_MAX_BYTES:
                    raise HTTPException(413, "Upload exceeds WELD_UPLOAD_MAX_BYTES")
                digest.update(chunk)
                stream.write(chunk)
        if size == 0:
            raise HTTPException(422, "Uploaded file is empty")
        metadata = {"file_id": identity, "filename": filename, "file_path": str(destination / name),
                    "size_bytes": size, "sha256": digest.hexdigest()}
        atomic_json(staging / "metadata.json", metadata)
        staging.rename(destination)
        return metadata
    finally:
        shutil.rmtree(staging, ignore_errors=True)


@router.delete("/files/{file_id}", status_code=204)
def delete_upload(file_id: str):
    """Delete only an owned upload. Call after the consuming job has finished."""
    if re.fullmatch(r"[a-f0-9]{32}", file_id) is None:
        raise HTTPException(404, "Upload not found")
    directory = settings.ARTIFACT_ROOT / "uploads" / file_id
    if not directory.is_dir():
        raise HTTPException(404, "Upload not found")
    shutil.rmtree(directory)
    return Response(status_code=204)


@router.get("/jobs/{job_id}/files/{kind}", response_class=FileResponse)
def download_result(job_id: str, kind: Literal["trajectory", "csv", "image"], request: Request):
    """Download a successful job's output; never accept arbitrary file paths."""
    try:
        job = request.app.state.jobs.get(job_id)
    except KeyError:
        raise HTTPException(404, "Job not found") from None
    if job["status"] != "succeeded":
        raise HTTPException(409, "Job has not succeeded")
    value = (job["result"] or {}).get(kind + "_path")
    if not value or not Path(value).is_file():
        raise HTTPException(404, "Job output not found")
    media_type = {"trajectory": "application/json", "csv": "text/csv", "image": "image/png"}[kind]
    return FileResponse(value, media_type=media_type, filename=Path(value).name)
