"""Strict public requests, matching the existing tool declarations."""
from typing import Annotated, Literal, Any
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

Number = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Positive = Annotated[Number, Field(gt=0)]
Vector3 = Annotated[list[Number], Field(min_length=3, max_length=3)]
Pose = Annotated[list[Number], Field(min_length=6, max_length=6)]
Matrix4 = Annotated[list[Annotated[list[Number], Field(min_length=4, max_length=4)]], Field(min_length=4, max_length=4)]


class Request(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_default=True)


class PointCloudRequest(Request):
    point_cloud_path: str = Field(min_length=1)
    input_frame: Literal["robot_base", "camera"]
    model_type: Literal["auto", "three_plane", "h_section", "square_tube"] = "auto"
    unit: Literal["mm", "m"] = "mm"
    camera_to_base: Matrix4 | None = None
    camera_position_base: Vector3 | None = None
    h_width_mm: Positive | None = None
    tube_width_mm: Positive | None = None
    max_planes: int = Field(default=20, ge=1, le=100, strict=True)
    min_seam_length_mm: Number = Field(default=3, ge=3)
    max_gap_mm: Positive | None = None
    voxel_size_mm: Number = Field(default=1, ge=0)
    sample_step_mm: Positive = 2
    posture_width_mm: Positive = 5
    approach_distance_mm: Positive = 10
    retract_distance_mm: Positive = 10
    plane_distance_threshold_mm: Positive = 2
    ransac_iterations: int = Field(default=100, ge=1, le=10000, strict=True)
    seed: int = Field(default=0, ge=0, le=4294967295, strict=True)
    tcp_rotation_offset_deg: Vector3 = Field(default_factory=lambda: [0, 0, 0])

    @model_validator(mode="after")
    def coordinates(self):
        from .point_cloud.io import rigid_transform
        if self.input_frame == "camera":
            if self.camera_to_base is None or self.camera_position_base is not None:
                raise ValueError("camera frame requires camera_to_base only")
            rigid_transform(self.camera_to_base)
        elif self.camera_to_base is not None:
            raise ValueError("robot_base must not specify camera_to_base")
        return self


class STLRequest(Request):
    stl_path: str = Field(min_length=1)
    unit: Literal["mm", "m"]
    pose_in_base: Pose
    seam_indices: list[Annotated[int, Field(ge=0, strict=True)]] | None = Field(default=None, min_length=1)
    min_seam_length_mm: Positive = 3
    sample_step_mm: Positive = 2
    approach_distance_mm: Positive = 10
    retract_distance_mm: Positive = 10

    @field_validator("seam_indices")
    @classmethod
    def unique(cls, value):
        if value is not None and len(value) != len(set(value)):
            raise ValueError("seam_indices must be unique")
        return value


class Workpiece(Request):
    stl_path: str = Field(min_length=1)
    unit: Literal["mm", "m"]
    pose_in_base: Pose


class SimulationRequest(Request):
    trajectory_path: str = Field(min_length=1)
    run_id: str = Field(min_length=1, pattern=r"\S")
    seam_id: str | None = Field(default=None, min_length=1, pattern=r"\S")
    tcp_offset_pose: Pose | None = None
    speed_mm_s: Positive = 20
    workpiece: Workpiece | None = None


class Response(BaseModel):
    model_config = ConfigDict(extra="allow")


class UploadedFile(Response):
    file_id: str
    filename: str
    file_path: str
    size_bytes: int
    sha256: str


class ExtractionResult(Response):
    trajectory_path: str
    csv_path: str
    image_path: str
    frame: Literal["robot_base"]
    seam_count: int
    executable_seam_count: int
    point_count: int
    warnings: list[str]
    seams: list[dict[str, Any]]
    statistics: dict[str, Any]
    intersection_quality: Any
    inline_artifact_arguments: dict[str, Any]


class SimulationArtifact(Response):
    type: Literal["robot_simulation"]
    title: str
    simulation_id: str
    url: str
    run_id: str
    summary: dict[str, int]
    tcp_calibrated: bool
    editable: Literal[False]
    mime_type: Literal["application/json"]


class JobResponse(Response):
    job_id: str
    kind: Literal["stl", "point-cloud", "simulation"]
    status: Literal["queued", "running", "succeeded", "failed", "cancelled"]
    result: ExtractionResult | SimulationArtifact | None
    error: dict[str, str] | None
    created: float
    updated: float
    status_url: str | None = None


class SimulationSnapshot(Response):
    schema_version: Literal[1]
    simulation_id: str
    run_id: str
    robot: str
    joint_names: list[str]
    joint_units: Literal["radian"]
    frame: Literal["robot_base"]
    units: dict[str, str]
    tcp_offset_pose: Pose
    tcp_calibrated: bool
    speed_mm_s: Positive
    source_path: str
    source_sha256: str
    input_snapshot: dict[str, Any]
    summary: dict[str, int]
    seams: list[dict[str, Any]]
