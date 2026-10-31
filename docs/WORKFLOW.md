# FR5 V6 STL 多焊缝规划与仿真执行原理

## 简版流程图

先运行 `execute:=false` 进行预规划与检查，人工确认后，再运行 `execute:=true` 执行。

![简版流程图](WORKFLOW_SIMPLE.png)

[SVG 矢量图](WORKFLOW_SIMPLE.svg) · [Graphviz 可编辑源文件](WORKFLOW_SIMPLE.dot)

参数对应关系：`start_weld_demo.sh` 使用 `execute:=false/true`；`start_weld_job.sh` 默认不执行，添加 `--execute` 开启执行。

## 详细流程

对应当前 `start_weld_job.sh` 实现。全部选中焊缝先通过规划检查，才允许开始运动。

```mermaid
flowchart TD
    subgraph INPUT[① 工件输入]
        A[输入 STL 文件<br/>单位 m/mm · 工件 XYZ/RPY · 最短焊缝长度]
        B[保存模型快照与配置<br/>记录 SHA-256 · 统一为米 · 校验网格]
        A --> B
    end

    subgraph GEOMETRY[② 焊缝识别与 TCP 路径生成]
        C[建立三角形共享边邻接<br/>计算二面角 · 判断凹角 · 按连通性聚类]
        D[按角度与长度筛选候选<br/>选择全部凹角候选或指定编号与顺序]
        E[沿焊缝等距采样<br/>由切向与两侧法向生成焊枪姿态<br/>增加接近段与撤离段]
        F{仅提取模式？}
        GEO[导出候选与 TCP 路径<br/>extracted_only]
        B --> C --> D --> E --> F
        F -- 是 --> GEO
    end

    subgraph PLAN[③ 场景同步与整批规划检查]
        G[启动 Gazebo、MoveIt、RViz<br/>固定初始角度：0,-120,120,-90,-90,0 度<br/>同一模型、尺度、位姿同步仿真与碰撞场景<br/>发布 world → workpiece_frame TF]
        H[逐条检查所有选中焊缝<br/>连续 IK · 关节限位与跳变检查<br/>加密碰撞检查 · FK 位置与姿态复核]
        I[生成轨迹时间戳<br/>RRTConnect 规划起点及焊缝之间的转移]
        J{整批检查全部通过？}
        V[RViz 显示候选、路径与规划轨迹<br/>保存各焊缝的规划结果]
        K{开启 --execute？}
        PREVIEW[仅规划和显示<br/>planned_only]
        F -- 否 --> G --> H --> I --> J
        J -- 是 --> V --> K
        K -- 否 --> PREVIEW
    end

    subgraph EXEC[④ 按顺序执行与反馈验证]
        L[执行当前焊缝<br/>转移到入口 → 接近 → 沿缝运动 → 撤离]
        M[MoveIt ExecuteTrajectory<br/>→ ros2_control 关节轨迹控制器<br/>→ Gazebo FR5 机械臂]
        N[读取 Action 终态和新鲜 joint_states<br/>核对关节到位与末端 TCP 误差]
        O{当前焊缝执行与反馈检查成功？}
        P{全部选中焊缝已完成？}
        DONE[记录全部执行成功<br/>completed_seams = total_seams<br/>succeeded]
        K -- 是 --> L --> M --> N --> O
        O -- 是 --> P
        P -- 否：下一条 --> L
        P -- 是 --> DONE
    end

    FAIL[停止后续运动<br/>记录失败焊缝、阶段和原因<br/>failed · 非零退出码]
    RESULTS[execution_result.json<br/>逐条状态、完成数量、终态误差]
    J -- 否 --> FAIL
    O -- 否 --> FAIL
    D -. 无有效候选 .-> FAIL
    E -. 无法生成有效姿态 .-> FAIL
    DONE --> RESULTS
    FAIL --> RESULTS
    PREVIEW --> RESULTS
    N -. 实际关节姿态同步显示 .-> V

    classDef input fill:#e6f1ff,stroke:#2563eb,color:#172554;
    classDef geometry fill:#e8faf1,stroke:#059669,color:#064e3b;
    classDef planning fill:#fff5df,stroke:#d97706,color:#78350f;
    classDef execution fill:#f1eaff,stroke:#7c3aed,color:#3b0764;
    classDef failure fill:#fff0f0,stroke:#dc2626,color:#7f1d1d;
    classDef success fill:#dcfce7,stroke:#16a34a,color:#14532d;
    class A,B input;
    class C,D,E,F,GEO geometry;
    class G,H,I,J,V,K,PREVIEW planning;
    class L,M,N,O,P execution;
    class FAIL failure;
    class DONE,RESULTS success;
```

## 数据与执行边界

- 候选与路径：`detected_seams.json`、`weld_path.csv`、`weld_path.yaml`，每条焊缝另存于 `seams/candidate_N/`。
- 默认只规划；显式添加 `--execute` 才发送运动命令。
- 每个执行 Action 必须成功且关节反馈到位，才发送下一条；取消、失败、终态不明或过期反馈均停止后续运动。
- “全部”指本次选中的几何候选。STL 没有焊接工艺语义，候选需要核对；当前执行的是仿真路径，不包含真实焊机起弧和送丝控制。
