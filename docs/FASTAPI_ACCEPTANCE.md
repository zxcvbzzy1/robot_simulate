# FastAPI 服务实施与验收记录

## 当前运行结果

- API 监听 `0.0.0.0:18800`，局域网 Swagger：`http://192.168.33.99:18800/docs`。
- MoveIt 接口宿主映射为 `0.0.0.0:18801`；容器 `fr5-weld-planner`。
- `/health` 与 `/ready` 均已实际验证。
- 已验证通过服务器局域网地址访问 API `/ready` 和容器 `/health`；也已从 Docker 网络经 `172.17.0.1:18800` 成功访问 API `/ready`。
- 原计划的 8800／8801 已被现有 `zzy` 容器的 8000–9000 映射占用，因此本机使用 18800／18801；没有停止或修改原容器。
- API 使用 conda `test`，Python 3.12.14。ROS／MoveIt 使用 ARM64 Docker，MoveIt 2.12.4。
- 依赖记录：`service/requirements.lock.txt`、`service/docker/ros-packages.lock.txt`、`service/docker/build-info.json`。

## 自动化检查

```bash
./scripts/weld_service.sh test
```

结果：**46 passed**。涵盖严格轨迹格式、旧 `poses` 格式、几何结果、单位／TCP 变换、重复 ID、非有限数值、64 MiB／50,000 点限制、PLY／PCD／XYZ、STL 默认及显式选择、实际算法子进程、输入快照、上传／下载、任务取消、超时、重启记录、单实例锁、宿主工具事件适配及 `.env` 自动加载。

测试依赖输出一条 Starlette TestClient 对 HTTPX 的弃用提示；未影响测试或实际 HTTP 调用。宿主 `domain/infra` 事件框架不在本工作区中，事件适配测试使用隔离桩；HTTP 工具客户端另有真实服务验证。

## 真实 MoveIt 验证

```bash
source output/service/runtime.env
conda run --no-capture-output -n test python service/tests/live_validation.py
```

末轮证据：`output/service/validation/live-report.json` 与同目录各场景 `*-snapshot.json`。

| 场景 | 结果 |
|---|---|
| 默认 200 mm TCP | 3／3 点通过 IK、FK、状态检查 |
| 显式零 TCP 偏移 | 3／3 点通过，确认默认值可覆盖 |
| 中间插入远距离目标 | 2 点通过，1 点失败，输出两个独立连续段 |
| 工件碰撞 | 障碍 STL 导致 3／3 点被拒绝并保留碰撞原因 |
| 移除工件后的下一任务 | 恢复为 3／3 点通过，确认场景清理 |
| STL 提取 → 仿真 | 1 条焊缝、5 个轨迹点全部通过 |
| 点云提取 → 仿真 | 3 条候选，其中 1 条有完整姿态、2 条仅几何；7 个轨迹点中末轮 5 点通过、2 点未找到 IK 解，诊断快照正常生成 |
| 运行中取消 ROS 任务 | 后台进程停止，任务标记 cancelled，后续轨迹正常求解 |

可达样例由独立 URDF 矩阵计算生成，再交给 MoveIt 求解；不是直接将已知关节值当作 API 结果。验证逐连续段检查时间递增、速度上界及时间参数化使用的加速度上界。点云中的局部 IK 失败属于所要求的诊断回放行为，不表示所有几何候选均可规划。

真实自碰撞及加密路径检查：

```bash
docker exec -e ROS_DOMAIN_ID=58 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  fr5-weld-planner bash -c \
  'source /opt/ros/jazzy/setup.bash; export PYTHONPATH="$WELD_WORKSPACE/service:$WELD_WORKSPACE/src/fr5v6_weld_planning:${PYTHONPATH:-}"; /usr/bin/python3 "$WELD_WORKSPACE/service/tests/ros_checks.py"'
```

结果：初始规划状态有效；实际发现 `j5_Link` 与 `base_link` 接触，IK 后碰撞检查拒绝该状态，加密路径检查也拒绝包含该状态的路径。证据：`output/service/validation/ros-checks.json`。

## 崩溃恢复与工具调用

```bash
source output/service/runtime.env
conda run --no-capture-output -n test python service/tests/live_recovery.py
```

此脚本会强制结束**本服务自己的 API**，仅用于验收，需先运行上一节生成输入。

实际结果：运行中任务在 API 重启后变为 `failed/interrupted`，遗留的宿主及 ROS 计算进程被终止，下一条任务 3／3 点求解成功。证据：`output/service/validation/recovery-report.json`。

两个工具的帮助模块已使用真实 HTTP 服务验证：点云入口完成提取；仿真入口生成产物后，现有 `read_simulation` 成功读取同一快照，仿真 ID、URL、run_id 和摘要保持原协议。证据：`tool-cloud-result.json`、`tool-simulation-report.json`，均位于上述 validation 目录。

容器已关闭 MoveGroup 执行 Action 能力；空闲状态查询 `/execute_trajectory` 的 Action server 数为 0。未启动 Gazebo、RViz、真实机器人连接或运动执行。

## 配置统一与仿真 503 修复

原代码默认把宿主规划服务地址设为 `127.0.0.1:8801`，实际 Docker 映射为 `18801`；手工运行 Uvicorn 不会读取旧 `runtime.env`，因此可能返回 503。

现已统一使用工作区 `.env`，默认规划 URL 为 `http://127.0.0.1:18801`。`python -m weld_service` 自动读取文件并按其中的地址／端口启动；从其他目录启动也能正确定位配置。显式外部环境变量保留优先级。

真实容器验证结果记录于 `output/service/validation/env-report.json`：`/ready` 返回 200；`POST /api/v1/simulations` 和 `POST /api/v1/simulations/` 均直接返回 202，两条任务都 succeeded，3／3 点通过检查。后台确实不可用时仍返回 503，并附实际规划 URL 及配置检查位置。

## 代码与 Git

工具基线提交：`b817fd8`，仅提交原工具源码、文档及模型；没有提交缓存、参考 HTML 或原有 package-lock。此后实现成果保持未暂存、未提交，原工作区的已有修改继续保留。

本次主要新增：`service/`、`scripts/weld_service.sh`、服务 API／验收文档与 OpenAPI。原工作区业务源码通过适配复用，未覆盖原运动执行器、URDF 或历史输出。工具侧修改了两个 builtin 入口、对应帮助入口，并新增共享 HTTP 客户端。

```bash
git -C /home/zzy/robot_simulate diff -- tool
git -C /home/zzy/robot_simulate status --short
git -C /home/zzy/robot_simulate/FR_Weld_Seam_Planning_ws status --short
```

完整宿主登录接口和前端代码未随本仓库提供，因此没有声称完成宿主页面的浏览器验收。接入时按服务文档配置共享快照目录与图片产物目录。
