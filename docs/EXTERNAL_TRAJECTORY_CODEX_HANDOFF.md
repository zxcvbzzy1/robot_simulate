# Codex 交接：外部焊缝轨迹导入、MoveIt 检查与 Gazebo 执行

请直接在现有项目中实现下述功能，完成构建、测试、运行实例和使用文档。不要只回答可行性，也不要重新搭建机器人工作区或比较多种规划器。

## 1. 用户目标与授权范围

用户已经通过其他算法提取出焊缝轨迹，希望把这些轨迹输入本项目，让 FR5 V6 在 Gazebo 中按顺序走完。

新增“外部轨迹输入”模式：

外部 CSV + YAML → Python 轨迹读取/校验 → TCP 路径 → MoveIt IK/碰撞/FK/时间参数化 → RViz 预览 → 用户显式开启 Gazebo 执行。

本次输入已经是轨迹，不需要重新用 STL 识别焊缝。STL 仍用于工件显示和碰撞场景。保留原有 STL 识别模式及其命令。

用户本次要求实现的功能是导入与仿真执行。不能推断为真实机器人、焊机 I/O 接入，也不能把原有几何候选自动当作已经确认的工艺焊缝。

## 2. 实际工作区与约束

- 实际项目：`/home/gsx/FR_Weld_Seam_Planning_ws`。
- Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2；先检查本机环境，不重复安装。
- ROS 环境：`ROS_DOMAIN_ID=58`，`GZ_PARTITION=fr5v6_weld_seam`。
- 使用 `scripts/with_weld_env.sh` 设置干净的覆盖层，使用 `scripts/build.sh` 构建。
- 原工作区 `/home/gsx/FR_Planning_Benchmark_ws`、`/home/gsx/FR_Gazebo_Jazzy_ws` 保持只读，不修改、不 source。
- 仓库存在大量未提交修改和未跟踪文件，这是当前项目已有成果。先检查 `git status`，保留所有既有工作，不 reset、不 checkout 覆盖、不清理输出目录。
- 算法继续采用 Python/rclpy，几何依赖保持 NumPy/PyYAML；不新增 trimesh、Open3D 或新规划器。
- 固定仿真初始关节角度：`[0,-120,120,-90,-90,0]` 度，不因导入外部轨迹而修改。
- 当前 TCP：`tool0 → weld_tcp` 固定变换，`xyz=[0,0,0.20] m`、`rpy=[0,0,0] rad`。这是用户要求从 150 mm 延长到 200 mm 后的值。
- `TEMPORARY_SIM_ONLY: true`。当前焊枪 TCP 是坐标偏移，没有实体长焊枪的完整碰撞模型。
- 既有历史任务快照可能仍含 150 mm TCP；不能复用旧轨迹或旧配置并声称已经与当前模型一致。

## 3. 已有代码：请复用这些能力

业务包：`src/fr5v6_weld_planning/fr5v6_weld_planning/`。

| 文件 | 当前作用 |
|---|---|
| `algorithm_nodes.py` | `/weld_seam_detector`、`/weld_path_generator` 两个 Python 服务节点 |
| `pipeline.py`、`pipeline_client.py`、`ros_codec.py` | 识别/生成服务、类型转换、配置及模型任务摘要校验 |
| `run_weld_demo.py` | `/weld_task_manager`，场景同步、算法调用、可视化及执行组织 |
| `batch.py` | 先整批预检，再逐条 transfer/weld；失败后停止后续运动 |
| `moveit_executor.py` | IK、FK、状态有效性、RRTConnect 转移规划、时间参数化、ExecuteTrajectory 和反馈检查 |
| `scene_manager.py` | 相同 STL、比例、位姿用于 MoveIt CollisionObject 与 Gazebo 工件 |
| `transforms.py` | RPY/四元数/刚体坐标变换，四元数顺序 xyzw |
| `pose_generator.py` | `WeldPathPoint`，现有姿态与 approach/seam/retreat 生成 |
| `workflow.py` | `PreparedGeometry`、多焊缝选择、世界坐标转换、路径速度时间计算 |
| `job_input.py`、`job_cli.py` | STL 任务输入、快照和现有命令行 |
| `simulation_session.py` | 同分区重复实例检查、专属进程标识和退出清理 |
| `output_writer.py`、`visualization.py` | JSON/CSV/YAML 输出、MarkerArray、DisplayTrajectory |
| `tcp_config.py` | 构建时从 YAML 同步 TCP 到复制后的 URDF |

已有接口包：`src/fr5v6_weld_interfaces/`。它只定义 ROS 消息和服务，业务实现仍为 Python。

已有 ROS 接口：`/compute_ik`、`/compute_fk`、`/check_state_validity`、`/plan_kinematic_path`、`/execute_trajectory`、`/joint_states`、`/apply_planning_scene` 及 Gazebo Spawn/Delete。

**状态说明：截至交接时，外部 CSV/YAML 导入入口尚未实现。** `start_weld_job.sh` 仅接受 STL 输入；已存在的 `weld_path.csv` 和 `weld_path.yaml` 是导出文件。不能把下述拟定命令描述成已经可运行。

现有 `GeneratePaths` 服务接收候选几何链，不直接接收任意外部 TCP 位姿，因此不能让外部轨迹假冒识别结果而导致重生成/覆盖输入姿态。

## 4. 第一版输入契约

第一版必须支持“完整 TCP 位姿”。CSV + YAML 分别保存点数据和任务配置。增加明确 schema_version，严格校验。CSV 内的位置指 `weld_tcp` 原点，不能解释成法兰位置。

### 4.1 CSV 示例

文件 `external_path.csv`，UTF-8，逗号分隔，第一行为表头：

```csv
seam_id,index,segment,x,y,z,qx,qy,qz,qw
seam_0,0,seam,0.000,0.000,0.000,0,0,0,1
seam_0,1,seam,0.005,0.000,0.000,0,0,0,1
seam_0,2,seam,0.010,0.000,0.000,0,0,0,1
```

该例仅说明格式，不能作为已验证可达的轨迹，也不能把单位四元数当作所有实际焊缝的通用姿态。

- `seam_id`：非空字符串，区分焊缝。
- `index`：每条焊缝内从 0 开始连续递增；执行顺序按文件行序，同一焊缝必须连续出现，禁止读取后静默排序。
- 焊缝顺序按 `seam_id` 首次出现顺序。
- `segment`：仅接受 `approach`、`seam`、`retreat`。
- `x,y,z`：位置；单位由 YAML 显式指定，导入后统一为米。
- `qx,qy,qz,qw`：单位四元数，描述 TCP 坐标轴相对 `frame_id` 的姿态，顺序固定 xyzw。
- 每条焊缝至少两个 seam 点。四元数必须有限、非零、接近单位长度；允许归一化小数舍入误差，严重偏离单位长度时报错。相邻四元数点积符号修正保持连续，不能改变实际朝向。
- 不静默填补缺失姿态、不跳过错误行、不丢弃失败点。

现有项目导出 CSV 还包含 `tx,ty,tz,n1x,n1y,n1z,n2x,n2y,n2z`，整批输出还有 `seam_id`。这些额外几何字段可作为兼容输入保留，但完整位姿模式下不应强制要求表面法向。

### 4.2 YAML 示例

文件 `external_job.yaml`：

```yaml
schema_version: 1
path_file: external_path.csv
frame_id: workpiece_frame
position_unit: m
orientation_format: quaternion_xyzw
tcp_frame: weld_tcp

workpiece:
  mesh: /home/gsx/stl/small3.stl
  mesh_unit: mm
  xyz: [-0.35, 0.1, 0.25]
  rpy_deg: [90, 0, 0]

path:
  generate_approach_retreat: true
  approach_distance_m: 0.05
  retreat_distance_m: 0.05
  seam_speed_mps: 0.01
```

- 相对 `path_file` 和 mesh 路径均相对该 YAML 的目录解析，不依赖启动目录。
- `position_unit` 与 `mesh_unit` 各自支持 `m`/`mm`，相互独立。
- `workpiece.xyz` 恒为米，`workpiece.rpy_deg` 为度。内部按现有配置转为弧度。
- `frame_id` 第一版支持 `workpiece_frame`、`world`。
- `workpiece_frame` 输入：位置和姿态都乘同一个工件刚体变换，随后用于世界坐标 IK。
- `world` 输入：不得再次施加工件位姿。工件模型仍按 `workpiece.xyz/rpy_deg` 放置用于碰撞检查。
- `tcp_frame` 必须对应当前 `weld_tcp`；启动时校验配置/机器人模型的 TCP 一致性。
- 相机等其他坐标系第一版明确拒绝，并提示需要标定变换；不要擅自当成 world。
- `generate_approach_retreat: true` 时 CSV 只含 seam 点，按当前工具 -Z 外侧方向补接近/撤离、等距采样并延续端点姿态。
- `generate_approach_retreat: false` 时每条路径必须提供完整、有序的 approach→seam→retreat 段，保留用户点序与姿态，不重复添加接近/撤离。
- 速度、距离、容差必须有限且有效。与主配置合并时只覆盖允许的字段，禁止通过输入悄悄关闭碰撞或终态检查。

如果现有 `world_path_poses()` 默认总是执行工件变换，应明确增加坐标系处理，不能直接沿用并导致 world 输入被变换两次。

### 4.3 仅 XYZ 的轨迹

第一版不是必需功能。XYZ 无法唯一确定焊枪姿态。缺少姿态时清楚报错；不要默认四元数为 `[0,0,0,1]`。后续可以另增“固定姿态”或“两侧法向生成姿态”模式，必须显式选择。

## 5. 实现要求

1. 增加可单独测试的 Python 导入器，负责读 CSV/YAML、单位转换、分段、多焊缝顺序、数值/姿态校验。
2. 在任务管理器加入外部轨迹来源分支，绕过识别与几何姿态生成，复用现有执行器和整批预检。可沿用现有三节点架构，不需要为读取文件重做所有 ROS 服务。
3. 建议新增 `scripts/start_weld_path.sh` 和对应 Python CLI，保留旧命令。输入为 `--config <external_job.yaml>`，默认预览，`--execute` 才执行；支持 `--no-rviz`、`--exit-when-done`、`--output`。
4. 复用原有环境隔离、Gazebo 同分区重复启动保护、专属进程清理和非零失败退出。
5. 快照保存输入 YAML、CSV、STL，计算 SHA-256，并记录实际 TCP、坐标系、单位、工件位姿、速度、最终生效配置。检测读取阶段资产变化。
6. 根据真实输入重构/复用路径数据类型。现有 `PreparedGeometry` 强耦合 STL 候选，外部位姿不一定有真实两侧法向；不要编造全零法向以假装数据齐全。可增加统一任务路径类型或使几何元数据可选，兼容既有 STL 模式。
7. 可视化显示工件、导入路径、TCP 轴、接近/撤离段、焊缝编号及 DisplayTrajectory。缺失表面法向时不画虚假的法向。
8. 导入路径不应调用现有由法向生成姿态的函数覆盖外部四元数。若添加采样，应保留原点顺序、端点，并明确姿态插值规则与新点记录。
9. 记录完整 execution_result.json：任务来源、预检状态、失败焊缝/段/点、MoveIt 错误码、Action 终态、完成数量、关节/TCP 误差、后续运动抑制。
10. 日志与退出码需要让调用方区分导入失败、场景同步失败、IK/限位/跳变/碰撞/FK/执行失败和未知终态。

## 6. 保留的执行检查

- 等待新鲜 joint_states 和所需服务/Action。
- 当前固定初始姿态校验；若修改该策略，必须明确解释，不能静默取消。
- 工件 STL 在 Gazebo 与 MoveIt 中使用相同单位、位姿和资产。
- 每个 TCP 点顺序 IK，以前一关节解为种子，使用 weld_tcp，碰撞检查开启。
- 关节有限值、URDF 限位、跳变检查；相邻关节按当前最大 0.02 rad 加密检查碰撞。
- seam 段 FK 检查位置、姿态和顺序，不跳过失败点。
- 合理速度/加速度和严格递增轨迹时间。
- 转移路径用现有 OMPL RRTConnect。用户点之间/焊缝之间不能直接发无碰撞验证的关节跳转。
- 先检查整批所有焊缝再发送运动。默认预览时不得调用 execute Action。
- 每次 Action 必须得到明确终态，且成功后才继续；取消、失败或终态未知时停止后续运动。
- TCP 变长不意味着真实工具碰撞模型已经完善，不能据此声称解决所有内部间隙问题。

## 7. 交付运行方式

实现以下拟定命令，并在最终文档标明它们已经过哪些实际验证：

```bash
cd /home/gsx/FR_Weld_Seam_Planning_ws
./scripts/build.sh

# 外部轨迹预览（未开启执行）
./scripts/start_weld_path.sh --config /absolute/path/external_job.yaml

# 关闭预览实例后，再显式执行
./scripts/start_weld_path.sh --config /absolute/path/external_job.yaml --execute
```

提供一套可复现实例 `examples/external_path/`：有效 CSV、YAML、格式说明。实例应从当前 TCP=200 mm 下重新验证通过的轨迹生成，而不是采用本文件的格式占位数值。

补充如何将其他 Python 算法输出的 positions/quaternions 写入文件的简短示例，明确坐标系和单位。

## 8. 验收与边界

- 测试真实解析、缺字段、非数值/NaN/Inf、零或无效四元数、乱序/重复 index、焊缝交错、段顺序、缺姿态和未知坐标系。
- 测试轨迹 mm→m 与模型 mm→m 独立转换；RPY=90° 的位置和姿态变换；world 模式不发生二次变换；四元数符号等价连续。
- 测试自动接近/撤离与显式段两种模式不重复生成，输入 seam 点不被覆盖。
- 测试缺少法向时执行和可视化仍合理，旧 STL 模式及输出兼容。
- 执行现有适用回归测试；交接前最近一次记录为 67 项通过，这仅是历史，不可当作本次修改验证。
- 构建后运行有效样例完整冷启动预览，检查 Gazebo/TF/PlanningScene/RViz/输出及 execute 未调用。
- 在预览通过后，于 Gazebo 中运行样例执行，确认顺序、Action 终态、最终误差；禁止连接实机。测试进程只清理自己启动的实例，不影响用户运行中的仿真。
- 至少一项无效输入与一项预检失败验证，确认非零退出、失败记录、零运动执行。
- 结束后再次确认可冷启动。若环境或样例执行失败，报告真实失败点，不得声称全闭环完成。

关于用户模型的历史事实：`small3.stl` 在若干工件位姿下 candidate_1 的 approach[7] 返回 -31；这只代表 IK 未找到解，未通过碰撞对象诊断证实根因。用户报告在 `xyz=[-0.35,0.1,0.25]`、RPY=[90,0,0]、选择 0,2,5,7 时可执行，报告发生在 TCP 延长前；不能把它视为当前 200 mm 下所有路径已经验证。

最终交付请说明改动文件、输入格式、实际运行命令、验证结果和未完成项。无需额外询问常规实现细节，按以上约定推进；只有输入数据或环境事实无法确定且影响正确性时再请求补充。
