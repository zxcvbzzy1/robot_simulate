# FR5 焊缝规划 Python 服务

本服务提供 STL／点云焊缝提取、统一轨迹文件和 MoveIt 规划结果。FastAPI 与算法运行于 conda `test`，ROS 2 Jazzy／MoveIt 运行于 ARM64 Docker 容器。现有前端消费仿真快照，服务不会执行机器人运动。

## 安装与手动启动

逐步命令见 [手动启动与跨机器文件调用](MANUAL_START.md)，包含 conda 依赖、Docker 创建／启动、前台运行 Uvicorn、文件上传与工具配置。无需调用启动 sh。

后端变量与默认值统一在工作区 `.env`，执行 `python -m weld_service` 自动加载。Docker 首次创建使用 `--env-file .env`，无需逐项 export。本机使用 API 18800、MoveIt 映射 18801；不要使用已被原有容器占用的 8800。工具访问远程服务时设置 `WELD_SERVICE_URL=http://100.82.31.1:18800`。

FastAPI 监听所有网卡并已开启 CORS。Swagger 为 `/docs`，OpenAPI 为 `/openapi.json`。只运行一个 API worker。

## 配置与文件共享

后端读取工作区 `.env`，显式的外部环境变量优先；修改后重启 API。下表是配置项说明，通常无需在命令行逐条设置。客户端工具的地址在其所在机器另行配置。

| 变量 | 默认值／用途 |
|---|---|
| `WELD_SERVICE_URL` | 工具访问 API，默认 `http://127.0.0.1:18800` |
| `WELD_PLANNER_URL` | API 访问容器，默认 `http://127.0.0.1:18801` |
| `WELD_PLANNER_READY_TIMEOUT_S` | 就绪探测超时，默认 15 秒 |
| `WELD_ENV_FILE` | 可选自定义配置文件，默认工作区 `.env`；相对路径以工作区为基准 |
| `WELD_BRIDGE_HOST/WELD_BRIDGE_PORT` | 容器内 HTTP 监听，默认 `0.0.0.0:8801` |
| `WELD_ARTIFACT_ROOT` | 默认工作区 `output/service`；任务数据库、日志、提取结果、输入副本 |
| `ROBOT_SIMULATION_ROOT` | 默认 `$WELD_ARTIFACT_ROOT/simulations`；API 与宿主快照读取器必须一致 |
| `WELD_INPUT_ROOT` | 可选：额外挂载一个外部输入目录，容器内保持原绝对路径 |
| `WELD_JOB_TIMEOUT_S` | 单任务超时，默认 3600 秒，API 和容器应一致 |
| `WELD_SERVICE_TIMEOUT_S` | 工具等待任务的超时，默认 3600 秒 |
| `WELD_CONTAINER_NAME` | 默认 `fr5-weld-planner` |
| `WELD_IMAGE` | 默认 `fr5-weld-planner:jazzy` |
| `WELD_UPLOAD_MAX_BYTES` | 服务端上传大小上限，默认 512 MiB |
| `WELD_CLIENT_ARTIFACT_ROOT` | 工具端 JSON／CSV／图片缓存目录，默认原宿主 `agent_flow/temp/point_cloud_welding` |
| `WELD_API_PORT` | API 监听端口，默认 18800 |
| `WELD_PLANNER_PORT` | 容器的宿主映射端口，默认 18801 |
| `WELD_API_HOST` | API 监听网卡，默认 `0.0.0.0` |
| `WELD_PLANNER_HOST` | Docker 端口映射绑定地址，默认 `0.0.0.0` |

输入可以是服务器已有文件路径；跨机器文件先通过 `POST /api/v1/files` 上传，再使用响应中的 `file_path`。上传保存在产物根目录下，已自动挂载进规划容器，无需挂载客户端目录。服务器另有输入目录时才需要 `WELD_INPUT_ROOT`。无需 GPU、图形显示或宿主 ROS。

工具帮助层会上传本地输入、下载提取结果，并在读取仿真快照时自动缓存服务端结果。工具与服务无需共享目录：`WELD_ARTIFACT_ROOT` 是服务端目录，`WELD_CLIENT_ARTIFACT_ROOT` 是工具端提取缓存，工具端的 `ROBOT_SIMULATION_ROOT` 是本地快照缓存。图片缓存须在原宿主 `agent_flow/temp` 下。这里的 `tool/` 只是宿主代码片段，完整登录路由和前端不在本仓库中。

## 接口

所有 JSON 字段拒绝未知参数，数值必须有限。耗时接口先返回 HTTP `202` 的任务对象：

```json
{
  "job_id": "32位十六进制ID",
  "kind": "simulation",
  "status": "queued",
  "result": null,
  "error": null,
  "created": 1790000000.0,
  "updated": 1790000000.0,
  "status_url": "/api/v1/jobs/32位十六进制ID"
}
```

| 接口 | 行为 |
|---|---|
| `GET /health` | 返回 API 存活状态，不依赖 MoveIt |
| `GET /ready` | 检查 MoveIt IK、FK、状态检查、规划及场景服务，以及产物目录写权限 |
| `POST /api/v1/files` | multipart 字段 `file` 上传，返回 file_id、服务端 file_path、大小与 SHA-256 |
| `DELETE /api/v1/files/{file_id}` | 清理临时上传，任务完成后调用；返回 204 |
| `GET /api/v1/jobs/{job_id}/files/{kind}` | 下载成功提取任务的 trajectory／csv／image |
| `POST /api/v1/extractions/point-cloud` | 点云焊缝提取 |
| `POST /api/v1/extractions/stl` | STL 焊缝提取 |
| `POST /api/v1/simulations` | 校验并快照输入轨迹，再排队请求 MoveIt |
| `GET /api/v1/jobs/{job_id}` | 返回任务状态、最终 `result` 或 `error` |
| `POST /api/v1/jobs/{job_id}/cancel` | 取消待运行任务或终止正在运行的计算；终态任务保持原状态 |
| `GET /api/v1/simulations/{simulation_id}` | 返回不可变仿真快照 |

状态依次为 `queued → running → succeeded/failed/cancelled`。服务重启后遗留任务记为 `failed`，错误码为 `interrupted`，不自动重试。取消规划时会等待容器内进程结束；无法确认时返回 `cancel_unconfirmed` 并暂停队列，直到容器确认空闲。

HTTP `404` 表示输入文件／任务／快照不存在，`422` 表示输入格式或参数非法，`503` 表示规划后端未就绪或存储不可写。提交后发生的算法、超时或运行故障记录在任务 `error={"code":"...","message":"..."}`。轨迹局部 IK／碰撞失败仍可成功生成诊断快照，不能仅用任务 `succeeded` 判断轨迹全部可达。

### 点云提取

参数与 `point_cloud_weld_seams` 工具一致，必填 `point_cloud_path` 和 `input_frame`：

```json
{
  "point_cloud_path": "/absolute/workpiece.pcd",
  "input_frame": "robot_base",
  "unit": "mm",
  "camera_position_base": [400, 300, 800]
}
```

- 支持 PLY、XYZ、ASCII／普通 binary PCD；不支持压缩 PCD。
- 相机坐标输入必须传 `camera_to_base` 右手刚性 4×4 变换，平移始终为 mm；不得同时传 `camera_position_base`。
- 基座输入不得再传 `camera_to_base`；不提供 `camera_position_base` 时仅输出几何。
- 默认：`max_planes=20`、`min_seam_length_mm=3`、`voxel_size_mm=1`、`sample_step_mm=2`、`posture_width_mm=5`、接近／退刀距离各 10 mm、平面阈值 2 mm、RANSAC 100 次、`seed=0`、TCP 旋转修正 `[0,0,0]`。
- `max_gap_mm` 默认自动计算；`model_type`、`h_width_mm`、`tube_width_mm` 保持旧参数兼容和弃用告警。完整范围及全部字段见 Swagger。

### STL 提取

```json
{
  "stl_path": "/absolute/workpiece.stl",
  "unit": "mm",
  "pose_in_base": [-550, 0, 250, 0, 0, 0],
  "sample_step_mm": 2,
  "approach_distance_mm": 10,
  "retract_distance_mm": 10,
  "min_seam_length_mm": 3
}
```

`stl_path/unit/pose_in_base` 必填。位姿格式 `[x,y,z,rx,ry,rz]` 为毫米／度，不随 STL 文件单位改变。省略 `seam_indices` 时选择全部凹焊缝；显式索引对应原算法稳定排序后的候选，允许选择非凹候选，索引不得重复。默认提取二面角范围为 45～135 度。候选仍需工艺确认。

两种提取的 `result` 均包含：

```json
{
  "trajectory_path": "/absolute/trajectory.json",
  "csv_path": "/absolute/trajectory.csv",
  "image_path": "/absolute/seams.png",
  "frame": "robot_base",
  "seam_count": 1,
  "executable_seam_count": 1,
  "point_count": 20,
  "warnings": [],
  "intersection_quality": null,
  "statistics": {},
  "seams": [],
  "inline_artifact_arguments": {"artifact_type": "image", "image": {"file_path": "/absolute/seams.png"}}
}
```

实际 `seams` 含 `id/start/end/length_mm/executable/quality`。点云保留其专有统计与交线诊断，STL 的 `intersection_quality` 为 `null`。`executable` 只表示姿态和轨迹可用；MoveIt 检查发生在下一步。合法输入无焊缝时返回空列表。

### 仿真

```json
{
  "trajectory_path": "/absolute/trajectory.json",
  "run_id": "宿主运行ID",
  "speed_mm_s": 20
}
```

- `trajectory_path/run_id` 必填；可选 `seam_id` 限制输出焊缝。
- 可选 `tcp_offset_pose=[x,y,z,rx,ry,rz]`，单位 mm／degree。省略为 `[0,0,200,0,0,0]`，`tcp_calibrated=false`、`TEMPORARY_SIM_ONLY=true`。显式传入时保留历史 `tcp_calibrated=true` 的兼容语义，表示调用方提供了偏移，服务不验证物理标定。
- 可选 `workpiece={"stl_path":"...","unit":"mm","pose_in_base":[0,0,0,0,0,0]}` 加入工件碰撞场景。默认仅检查机器人自碰撞；没有实体焊枪碰撞模型。
- 输入不超过 64 MiB，总点数不超过 50,000；选择单缝时也校验整个文件。
- MoveIt 输出关节顺序 `j1..j6`、单位弧度，求解后用 FK 校验位置 ≤1 mm、姿态 ≤1°；关节跳变阈值沿用工作区 0.5 rad，加密检查步长 0.02 rad。
- 时间参数化结合 TCP 速度、关节速度及加速度限制；该时间是规划结果，不代表物理执行反馈。

成功任务的 `result` 是现有内联产物描述，包含 `simulation_id/run_id/url/summary/tcp_calibrated`。快照保留 `seams[].points[]` 的 `pose/stage/joints/status/time_s/minimum_duration_s` 和 `segments[]` 的 `start/end/duration_s`。失败点 `joints=null/status=unreachable` 并记录原因；每个连续段重新从 0 计时，不跨段插值。

快照还包含 `joint_velocity_limits_rad_s` 和 `joint_acceleration_limits_rad_s2`，供核对时间约束。快照额外包含 `planning` 检查范围、模型摘要、初始状态、`executed=false` 及 `transfers`。转移路径独立存放，不混入旧前端阶段枚举。若现有前端不读取新字段，原有焊缝回放仍可使用。

## Python 完整调用示例

```python
import os
import time
import httpx

with httpx.Client(base_url=os.getenv("WELD_SERVICE_URL", "http://127.0.0.1:18800"), timeout=30, trust_env=False) as api:
    def wait(response):
        response.raise_for_status()
        job_id = response.json()["job_id"]
        while True:
            response = api.get(f"/api/v1/jobs/{job_id}")
            response.raise_for_status()
            job = response.json()
            if job["status"] == "succeeded":
                return job["result"]
            if job["status"] in {"failed", "cancelled"}:
                raise RuntimeError(job["error"] or job["status"])
            time.sleep(0.2)

    with open("/本机实际路径/workpiece.xyz", "rb") as stream:
        response = api.post("/api/v1/files", files={"file": ("workpiece.xyz", stream)})
    response.raise_for_status()
    uploaded = response.json()
    extraction = wait(api.post("/api/v1/extractions/point-cloud", json={
        "point_cloud_path": uploaded["file_path"],
        "input_frame": "robot_base", "camera_position_base": [400, 300, 800],
    }))
    api.delete(f"/api/v1/files/{uploaded['file_id']}").raise_for_status()
    artifact = wait(api.post("/api/v1/simulations", json={
        "trajectory_path": extraction["trajectory_path"], "run_id": "example-run",
    }))
    snapshot = api.get(f"/api/v1/simulations/{artifact['simulation_id']}").json()
    print(snapshot["summary"])
```

```bash
WELD_SERVICE_URL="${WELD_SERVICE_URL:-http://127.0.0.1:18800}"
curl -f "$WELD_SERVICE_URL/ready"
curl -f -H 'Content-Type: application/json' \
  -d '{"trajectory_path":"/absolute/trajectory.json","run_id":"example-run"}' \
  "$WELD_SERVICE_URL/api/v1/simulations"
curl -f "$WELD_SERVICE_URL/api/v1/jobs/JOB_ID"
curl -f -X POST "$WELD_SERVICE_URL/api/v1/jobs/JOB_ID/cancel"
```

## 工具接入与排错

部署本次修改的 builtin 工具、`tool/help/weld_service_client.py`、`tool/help/weld_service_files.py`、`tool/help/robot_simulation.py` 和点云包入口至原宿主对应位置。工具自行提交、轮询任务并转发取消；`robot_simulate_trajectory` 仍自动发布原有内联产物事件，无需前端改动。

- `/ready` 或仿真提交返回 503：先检查错误中的实际规划地址，必须是宿主映射 `http://127.0.0.1:18801`，不是容器内部 8801。检查 `.env`、启动日志和旧终端中覆盖配置的同名环境变量；执行 `curl http://127.0.0.1:18801/ready` 核对容器状态，再查看 `docker logs fr5-weld-planner`。
- 任务失败：检查 `$WELD_ARTIFACT_ROOT/jobs/JOB_ID/worker.log`；规划详情在 `planner/JOB_ID/planner.log`。
- 文件不存在：客户端本地文件先上传，后续传服务返回的 `file_path`；工具运行机器本身读不到的路径无法自动上传。服务端已有文件才需要检查容器挂载。
- 工具已成功但前端读不到快照：确认宿主已部署新版 `help/robot_simulation.py`，`WELD_SERVICE_URL` 指向 18800，工具端 `ROBOT_SIMULATION_ROOT` 可写，运行上下文和宿主身份检查匹配。
- 图片产物目录被拒绝：将工具端 `WELD_CLIENT_ARTIFACT_ROOT` 设置到宿主 `agent_flow/temp` 下；不需要修改服务端产物目录。
- 上游 MoveIt 初始化日志可能提示未配置控制器或在线 3D 传感器。本服务关闭了执行 Action、未加载在线感知；使用 `/ready` 与任务结果判断规划状态。

## Git 差异

工具基线提交：`b817fd8`。实施成果不自动提交。

```bash
git -C /home/zzy/robot_simulate diff -- tool
git -C /home/zzy/robot_simulate status --short
git -C /home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws diff
git -C /home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws status --short
```

新增服务文件在工作区仓库中显示为未跟踪；原工作区已有未提交修改继续保留。源码之外的任务、日志和验证产物保存在已忽略的 `output/`。
