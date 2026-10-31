# Python 节点结构与运行实例

## 当前结构

三个独立进程均使用 Python / rclpy；不是把原有脚本简单改名。

| 节点 | 职责 | 接口 |
|---|---|---|
| `/weld_seam_detector` | 读取 STL、提取共享边、筛选和排序候选焊缝 | `/weld/detect_seams` 服务 |
| `/weld_path_generator` | 选择焊缝、等距采样、生成姿态和 approach/seam/retreat | `/weld/generate_paths` 服务 |
| `/weld_task_manager` | 调用前两个节点、同步场景、整批安全检查、可视化、按顺序执行及记录 | MoveIt 服务/Action、`/joint_states` |

数据流：STL → 识别节点 → 候选链 → 路径节点 → TCP 路径 → 任务管理 → MoveIt → ros2_control → Gazebo。
任务管理器顺序调用两个服务，不依赖 Topic 到达顺序。算法节点本身没有机器人执行接口。

`fr5v6_weld_interfaces` 只定义消息和服务，构建时自动生成 Python 类型；不包含自写 C++ 算法。
MoveIt、ros2_control 和 Gazebo 保持原有实现，不重写成 Python，也不由 Python 承担实时伺服。
`run_weld_demo` 是任务管理入口的兼容别名；单独运行它之前需要启动两个算法节点。

## 代码位置

以下路径均相对本工作区 `/home/gsx/FR_Weld_Seam_Planning_ws`：

- `src/fr5v6_weld_planning/fr5v6_weld_planning/algorithm_nodes.py`：两个算法节点。
- `pipeline.py`、`ros_codec.py`：消息转换、任务标识和数据校验。
- `pipeline_client.py`：任务管理器调用算法服务。
- `run_weld_demo.py`、`batch.py`：任务管理、预检、顺序执行和故障记录。
- `mesh_loader.py`、`seam_extractor.py`、`seam_processor.py`、`pose_generator.py`：纯 Python 算法，仍可离线调用。
- `src/fr5v6_weld_interfaces/msg/`、`srv/`：类型化 ROS 数据契约。

## 构建

```bash
cd /home/gsx/FR_Weld_Seam_Planning_ws
./scripts/build.sh
```

启动脚本会自动设置干净的 ROS 环境、`ROS_DOMAIN_ID=58`、`GZ_PARTITION=fr5v6_weld_seam`。
不要同时启动两个完整仿真实例。固定初始角度仍为 `[0,-120,120,-90,-90,0]` 度。

## 示例一：原有演示入口

```bash
./scripts/start_weld_demo.sh execute:=false
```

三个 Python 节点会自动启动。此时执行识别、路径生成、IK/FK/碰撞检查和 RViz 显示，不发送运动 Action。
确认预览与结果后，在原终端 Ctrl+C 关闭完整仿真，再重新启动：

```bash
./scripts/start_weld_demo.sh execute:=true
```

`execute:=true` 仍重新进行安全检查，检查通过后才允许在 Gazebo 中运动。
默认演示按 `candidate_index` 选择一条焊缝；多焊缝使用下面的 job 入口。

## 示例二：样例工件的全部凹角候选焊缝

预览两条焊缝：

```bash
./scripts/start_weld_job.sh \
  --mesh /home/gsx/FR_Weld_Seam_Planning_ws/src/fr5v6_weld_planning/meshes/workpiece.stl \
  --units m --xyz -0.55 0 0.25
```

关闭上述仿真，确认无误后执行：

```bash
./scripts/start_weld_job.sh \
  --mesh /home/gsx/FR_Weld_Seam_Planning_ws/src/fr5v6_weld_planning/meshes/workpiece.stl \
  --units m --xyz -0.55 0 0.25 --execute
```

`--execute` 对应 launch 的 `execute:=true`；省略它就是 `execute:=false`。
可加 `--exit-when-done` 自动关闭本次仿真；可加 `--no-rviz` 不打开 RViz。
每次运行生成独立的 `output/jobs/<时间戳>/`，包含输入快照、配置、候选、路径和执行结果。

## 示例三：你的 small.stl

先做离线几何识别，不启动 Gazebo：

```bash
./scripts/start_weld_job.sh \
  --mesh /home/gsx/stl/small.stl \
  --units mm --xyz -0.55 0 0.25 --min-length 0.005 --extract-only
```

然后用三节点结构预览及 MoveIt 检查：

```bash
./scripts/start_weld_job.sh \
  --mesh /home/gsx/stl/small.stl \
  --units mm --xyz -0.55 0 0.25 --min-length 0.005
```

只有该模型预览和安全检查通过后，才在同样参数末尾加 `--execute`。
这里 `--xyz` 是米制世界坐标的三个实际数值，不能输入字母 `X Y Z`。
`--units mm` 仅说明 STL 顶点单位；`--min-length 0.005` 仍为米（5 mm）。
工件放置坐标是仿真起始示例，不保证每条焊缝都可达。

## 单独启动节点，便于修改 Python 算法

如果只调试算法服务，不需要 Gazebo：

```bash
./scripts/with_weld_env.sh ros2 launch fr5v6_weld_planning weld_algorithms.launch.py
```

或者分别在两个终端执行：

```bash
./scripts/with_weld_env.sh ros2 run fr5v6_weld_planning weld_seam_detector
./scripts/with_weld_env.sh ros2 run fr5v6_weld_planning weld_path_generator
```

以上两种启动方式二选一；完整 demo/job 已自动启动它们，不要再重复启动。
查看接口与节点：

```bash
./scripts/with_weld_env.sh ros2 node list
./scripts/with_weld_env.sh ros2 service list -t
./scripts/with_weld_env.sh ros2 interface show fr5v6_weld_interfaces/srv/DetectSeams
./scripts/with_weld_env.sh /usr/bin/python3 -m pytest -q src/fr5v6_weld_planning/test
```

## 数据与安全边界

- 传输单位为米，路径和候选位于 `workpiece_frame`；任务管理器转换到世界坐标用于 MoveIt。
- 保留候选编号、链拓扑、两侧法向、采样点、四元数、工具轴和段标签，不使用单独的 PoseArray 丢弃这些字段。
- `job_id` 是配置文件的 SHA-256；各阶段检查配置及 STL 摘要，拒绝混用不同任务的数据。
- 服务采用本机共享文件路径，不是上传文件服务，尚不支持不同机器上的任意路径直接互通。
- 服务失联、超时、返回失败、缺少焊缝或无效数值都会终止任务，不退回旧的本地算法偷偷继续。
- 多焊缝任务仍先检查全部路径，再逐条执行；任何失败停止后续运动。预览并不自动授权执行，用户仍须显式开启执行。
- “全部焊缝”目前指算法识别并选中的有效候选，不等于任意 CAD/STL 上所有工艺焊缝。STL 几何本身不包含焊接工艺语义。
- 目前输入格式仍是 ASCII/Binary STL；STEP/IGES/OBJ 未新增支持。
- 仅 Gazebo 仿真；尚未接入真实焊机 I/O。TCP、工件标定、焊枪模型及实机安全联锁仍需另行验证。

## 本次验证记录（2026-10-08）

- 完整工作区构建：5 个包成功。
- Python 回归测试：67 项通过，含隔离域 59 中的真实 ROS 服务调用及错误响应测试。
- 多焊缝预览：`output/jobs/20261008_171530_812616/`，`planned_only`，2 条焊缝、122 个点，没有运动执行。
- 多焊缝执行（启动 RViz）：`output/jobs/20261008_171716_269398/`，`succeeded`，完成 2/2 条，4 个 Action 均为 `SUCCEEDED`。两条路径结束位置误差约 0.0038 mm、0.0058 mm。
- 原 `start_weld_demo.sh execute:=false` 入口：单焊缝、61 点预览通过。
- `small.stl`：`output/jobs/20261008_171839_194473/`，离线识别 8 条；本次未验证该模型的 MoveIt 可达性或执行，不能据此认定它能全部焊完。
- 验证后确认同分区没有遗留 Gazebo 进程。Gazebo GUI 关闭时出现 SIGINT 超时后 SIGTERM 的已有退出日志，专属进程清理已完成，不影响上述 Action 终态。
