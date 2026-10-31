# 导入工件并执行多条焊缝

工作区：`/home/gsx/FR_Weld_Seam_Planning_ws`。先运行 `./scripts/build.sh`。

## 输入与坐标

第一版接受 ASCII/Binary STL。STEP、IGES、OBJ 等请先在 CAD 软件中导出 STL。
STL 不记录单位，`--units m` 或 `--units mm` 必须明确提供。
`--xyz X Y Z` 是 **STL 原点在 world 中的位置，单位米**；`--rpy-deg R P Y` 是该工件的旋转，单位度。
程序不会自动把模型居中、缩到机械臂工作范围内或标定其位置。

下面的位姿是已验证的项目 T 型示例参数。自己的模型必须使用自己的放置位置。

```bash
cd /home/gsx/FR_Weld_Seam_Planning_ws
./scripts/start_weld_job.sh \
  --mesh src/fr5v6_weld_planning/meshes/workpiece.stl \
  --units m --xyz -0.55 0 0.25 --rpy-deg 0 0 0
```

该命令启动 Gazebo、MoveIt 和 RViz，识别所有有效凹角候选并预规划全部轨迹，不执行机械臂运动。
RViz 展示候选、选中焊缝、执行顺序编号、采样点、工具轴以及接近和撤离段。
机器人启动角度仍为 `[0,-120,120,-90,-90,0]` 度。

## 执行与候选选择

确认工件位置和识别结果后，在命令末尾添加 `--execute`，会新建一次任务并在 Gazebo 执行。
不要同时启动两个使用该工作区隔离域的演示；先关闭上一轮。

- 默认 `--seams all`：选择所有满足当前几何阈值的**凹角候选**，按稳定候选编号执行。
- `--seams 1,0`：只处理这两个候选，执行顺序为 1 然后 0；可显式选择凸边候选。
- `--min-length 0.02`：最短焊缝阈值为 20 mm（此参数始终用米）；默认 0.15 m。
- `--extract-only`：仅提取和导出，不启动 Gazebo/MoveIt。
- `--no-rviz --exit-when-done`：自动验证用，任务结束关闭仿真并返回退出码。
- `--output /absolute/new_directory`：输出到新的或空的目录，避免混入上一次结果。

整个任务首先完成所有焊缝的 IK、加密碰撞检查、FK 复核和焊缝间的 RRTConnect 转移规划。
全部预检查通过才开始执行。每条焊缝按“转移到入口 → 接近 → 焊缝 → 撤离”完成。
下一条运动必须等待前一条成功及终态反馈稳定。失败、取消、未知终态或反馈过期都会停止后续动作。
不自动跳过不可达焊缝，不自动回零，不自动重试已经执行过的焊缝。

## 结果

终端打印任务目录：`output/jobs/时间戳/`。

- `input.stl`：本次输入的独立快照。
- `job_config.yaml`：单位、位姿、候选选择和输入 SHA-256。
- `detected_seams.json`：全部候选及本次选中的焊缝编号。
- `weld_path.csv` / `.yaml`：按焊缝分组的 TCP 路径，增加 `seam_id` 字段；index 在每条焊缝内从零计数。
- `seams/candidate_N/`：每条焊缝的独立路径文件。
- `execution_result.json`：整批和各条焊缝的规划/执行状态、完成数量、失败阶段、终态误差。

`planned_only` 只代表规划成功；只有 `succeeded` 且 `completed_seams == total_seams` 才表示本次选择的全部焊缝在仿真中执行成功。
`extracted_only` 不表示 IK 或碰撞检查通过。

## 当前识别范围

凹角来自共享三角形边的二面角、长度及连通性。STL 本身不包含焊接工艺、接头定义或焊机 I/O，
因此不能从任意 CAD 外形自动保证找到所有真实焊缝。未焊接的棱边也可能被识别为候选。
需要检查 RViz/候选文件，必要时使用 `--seams` 指定列表。
分离零件之间没有共享网格边的接缝、带间隙接头、法向混乱、非流形网格以及超出工作空间的工件，
不属于“自动成功”的保证范围；不能可靠生成路径时会报错。
闭环候选包含闭合边；急转弯或工具姿态不连续的路径仍会被姿态检查拒绝。

本流程完成的是仿真中的焊枪路径运动，不控制真实焊机起弧、送丝或实机安全联锁。
