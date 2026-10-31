# 手动启动与跨机器文件调用

下面逐条执行命令，不调用 `weld_service.sh`。服务端使用 18800，容器映射使用 18801；**工具不要再连接 `100.82.31.1:8800`**。

这次更新没有替你停止、启动或重启现有服务。新增上传接口在你重新启动 API 后生效。

## 1. 服务端：准备环境

在部署机器的终端执行：

```bash
cd /home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws
source /root/miniconda3/etc/profile.d/conda.sh
conda activate test
python -m pip install -r service/requirements.lock.txt
python -m pip install --no-deps --no-build-isolation -e ./service
```

依赖增加了 `python-multipart`，用于接收文件。若官方源访问超时，安装命令增加 `-i https://pypi.tuna.tsinghua.edu.cn/simple`。

后端配置已集中在工作区根目录 `.env`，启动时自动读取，不需要逐条 export。默认 API 18800、MoveIt 宿主映射 18801、存储目录 `output/service`。若移动部署目录，只需修改 `.env` 中的路径。

```bash
mkdir -p output/service
```

文件按模块位置查找，不依赖当前启动目录。外部进程环境变量若存在，会覆盖 `.env`；若旧终端曾设置错误地址，可换新终端，或执行一次 `unset WELD_PLANNER_URL`。

## 2. 服务端：准备 MoveIt 镜像

先检查是否已有镜像：

```bash
docker image ls fr5-weld-planner:jazzy
```

已有可直接进入下一步；首次部署才构建：

```bash
docker build -t fr5-weld-planner:jazzy service/docker
```

本次 API 的 `.env` 读取与端口修复不要求重建已有 MoveIt 镜像。首次构建的镜像入口也支持通过 `.env` 指定 ROS 隔离配置。

## 3. 服务端：启动容器

先看容器是否已存在：

```bash
docker ps -a --filter name=fr5-weld-planner
```

如果已有该容器且之前按本项目配置创建，执行：

```bash
docker start fr5-weld-planner
```

只有首次创建容器时执行下面的命令；不要对同名已有容器重复 `docker run`：

```bash
docker run -d --init --name fr5-weld-planner --restart unless-stopped \
  --env-file .env \
  -p 0.0.0.0:18801:8801 \
  -v /home/zzy/robot_simulate:/home/zzy/robot_simulate:ro \
  -v /home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws/output/service:/home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws/output/service:rw \
  fr5-weld-planner:jazzy
```

`--env-file .env` 统一传入容器配置。若修改端口或挂载路径，重新创建容器以应用配置。容器自动运行 MoveIt 规划节点和 HTTP 适配层。检查状态：

```bash
docker logs --tail 40 fr5-weld-planner
curl --noproxy '*' -f http://127.0.0.1:18801/ready
```

应返回 `{"status":"ready","backend":"MoveIt2"}`。首次启动可能需等待几秒。

## 4. 服务端：启动 FastAPI

如果之前的 API 仍在运行，先在原终端按 Ctrl+C。若之前在后台启动，可查询进程，确认是本项目后终止该 PID：

```bash
ss -ltnp 'sport = :18800'
ps -p 实际PID -o pid,args
kill 实际PID
```

在已激活 `test` 的终端前台运行，API 地址、端口、规划地址和路径均自动取自 `.env`：

```bash
python -m weld_service
```

保持此终端运行。日志会显示使用的 `.env` 和实际 MoveIt 地址，应为 `http://127.0.0.1:18801`。API 停止时按 Ctrl+C。只启动一个 worker，否则任务目录锁会拒绝第二个实例。

在另一个服务端终端检查：

```bash
curl --noproxy '*' -f http://127.0.0.1:18800/health
curl --noproxy '*' -f http://127.0.0.1:18800/ready
```

前者验证 API；后者验证 API 能调用容器内的 MoveIt。浏览器打开 `http://100.82.31.1:18800/docs`，或将地址替换为服务器当前局域网 IP。`100.82.31.1` 是当前服务器的 Tailscale 地址，客户端需能访问该网络。

## 5. 文件所在机器：先上传，再提交任务

**本地路径属于哪台机器，文件上传就必须由能读到该文件的机器执行。** 服务器无法直接打开另一台机器的 `/Users/...` 或 `C:\...`。

在文件所在机器执行：

```bash
export WELD_SERVICE_URL=http://100.82.31.1:18800
curl -f -F 'file=@/这台机器上的实际路径/workpiece.ply' \
  "$WELD_SERVICE_URL/api/v1/files"
```

支持 `.ply/.pcd/.xyz/.stl/.json`，默认单文件不超过 512 MiB。返回示例：

```json
{
  "file_id": "32位上传ID",
  "filename": "workpiece.ply",
  "file_path": "/home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws/output/service/uploads/上传ID/input.ply",
  "size_bytes": 12345,
  "sha256": "..."
}
```

复制响应中真实的 `file_path`，作为点云任务的 `point_cloud_path`：

```bash
curl -f -H 'Content-Type: application/json' \
  -d '{"point_cloud_path":"这里替换为上传返回的file_path","input_frame":"robot_base","unit":"mm","camera_position_base":[400,300,800]}' \
  "$WELD_SERVICE_URL/api/v1/extractions/point-cloud"
```

`camera_position_base` 应换成实际扫描位置；缺少时可以不传，但只会生成几何结果。相机坐标输入需改用 `input_frame=camera` 并提供实际 `camera_to_base`。

任务返回 `job_id`，随后查询：

```bash
curl -f "$WELD_SERVICE_URL/api/v1/jobs/实际job_id"
```

等待 `status=succeeded`。此时 `result.trajectory_path` 是**服务端生成的路径**，可直接传给同一服务的仿真接口，不用再次上传：

```bash
curl -f -H 'Content-Type: application/json' \
  -d '{"trajectory_path":"这里替换为result.trajectory_path","run_id":"your-run-id"}' \
  "$WELD_SERVICE_URL/api/v1/simulations"
```

如需下载提取结果到当前机器：

```bash
curl -f "$WELD_SERVICE_URL/api/v1/jobs/提取任务job_id/files/trajectory" -o trajectory.json
curl -f "$WELD_SERVICE_URL/api/v1/jobs/提取任务job_id/files/csv" -o trajectory.csv
curl -f "$WELD_SERVICE_URL/api/v1/jobs/提取任务job_id/files/image" -o seams.png
```

使用上传文件的任务完成后，清理临时输入：

```bash
curl -f -X DELETE "$WELD_SERVICE_URL/api/v1/files/上传返回的file_id"
```

删除上传文件不会删除已经生成的提取结果和仿真快照。不要在任务仍处于 `queued/running` 时清理其输入。手工上传默认保留到显式删除，避免排队期间被自动过期清理。

## 6. 使用原工具时的自动流程

在**工具进程所在机器**设置：

```bash
export WELD_SERVICE_URL=http://100.82.31.1:18800
```

随后从这个环境启动 Agent 应用；只在另一个终端设置变量，不会改变已运行进程的环境。

部署更新后的 `tool/help/weld_service_client.py`、新增的 `tool/help/weld_service_files.py` 和更新后的 `tool/help/robot_simulation.py`。builtin 继续只负责参数绑定和事件发布。

帮助层会执行：

1. 路径在工具机器上可读：流式上传文件，替换成返回的服务端路径。
2. 路径只存在于服务端：保持原路径传给服务，兼容上一任务的返回值。
3. 等待提取完成，将 JSON／CSV／PNG 下载至工具机器的临时产物目录，保持原有本地路径返回格式。
4. 仿真快照本地缺失时通过 API 获取并缓存，供原有宿主读取器使用。
5. 任务完成或确认取消后清理本次上传的输入；无法确认后台终止时保留输入。

`WELD_CLIENT_ARTIFACT_ROOT` 可覆盖工具端产物缓存，默认位于原宿主 `agent_flow/temp/point_cloud_welding`；`ROBOT_SIMULATION_ROOT` 配置工具端仿真快照缓存。两台机器不再要求这些目录共享或具有相同路径。图片缓存仍需放在宿主允许的 `agent_flow/temp` 下。

如果文件只在浏览器所在电脑，而工具运行在第三台机器，仍需由浏览器用 multipart 上传或在文件电脑执行上述 curl。只传一个第三台机器读不到的路径，工具无法自动取得文件内容。
