# FastAPI service

新增入口：STL／点云统一提取 → 容器内 MoveIt 规划 → 现有前端回放。API 使用 conda `test`，不依赖 Gazebo 渲染。逐条启动命令与远程文件上传见 [手动启动](docs/MANUAL_START.md)，接口见 [FastAPI 服务文档](docs/FASTAPI_SERVICE.md)。以下为保留的原 ROS 项目说明。

# FR5 V6 STL Weld Seam Planning

The application now runs as three independent Python/rclpy nodes: seam detector,
path generator, and task manager. See [Python nodes and runnable examples](docs/PYTHON_NODES.md).
The MoveIt / ros2_control / Gazebo backend is unchanged.

ROS 2 Jazzy / Gazebo Harmonic simulation project for extracting a straight
fillet-weld candidate from a clean STL, generating a continuous TCP path,
validating it with MoveIt, and optionally executing it on the FR5 V6 model.
External STL input, explicit millimetre/metre units, and complete multi-seam jobs
are available through `./scripts/start_weld_job.sh`; see
[the user model guide](docs/USER_STL_JOBS.md).
The three reused packages were copied from the accepted
`FR_Planning_Benchmark_ws` commit `ebdc9d3`; that source workspace and
`FR_Gazebo_Jazzy_ws` are not modified or sourced at runtime.

```bash
cd /home/gsx/FR_Weld_Seam_Planning_ws
./scripts/build.sh
./scripts/start_weld_demo.sh execute:=false
# Only after reviewing RViz and output/execution_result.json:
./scripts/start_weld_demo.sh execute:=true
```

The launch uses `ROS_DOMAIN_ID=58` and `GZ_PARTITION=fr5v6_weld_seam` and
writes `detected_seams.json`, `weld_path.csv`, `weld_path.yaml`, and
`execution_result.json` under `output/`. Geometry and safety parameters are in
`src/fr5v6_weld_planning/config/weld_project.yaml`.

The default 200 mm tool-Z TCP extension is explicitly `TEMPORARY_SIM_ONLY` and
exists only to keep the simulated flange clear of the coupon. Before real-robot
use, replace the TCP calibration, torch geometry, workpiece pose,
payload, manufacturer dynamics, collision margins, safety interlocks and weld I/O.
