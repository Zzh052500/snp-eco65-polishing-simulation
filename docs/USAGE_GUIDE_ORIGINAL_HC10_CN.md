# 原版(Motoman hc10)打磨仿真使用说明

> 适用对象:本仓库**替换为睿尔曼 eco65 之前**的版本,即 git HEAD 对应的 hc10 配置
> (机器人 Motoman hc10 + RealSense 相机,打磨/抛光仿真)。若你拿到的是还没做 eco65
> 替换的干净仓库,按本文操作即可。

## 1. 这是什么

本 Demo 基于 ROS Industrial 的 **SNP Automate 2023**:
一台装在桌子上的 Motoman HC10,配合 Intel RealSense 相机,对任意工件做
**表面三维重建 → 栅格(raster)工具路径规划 → 机器人打磨运动规划** 的完整软件流程演示。

- 本文描述**仿真模式**(不连真实机器人、不做真实抛光加工)。
- 完整英文原版说明见 [`docs/ORIGINAL_README.md`](ORIGINAL_README.md);
  详细逐按钮中文流程见 [`docs/PROJECT_WORKFLOW_CN.md`](PROJECT_WORKFLOW_CN.md)。

## 2. 工作流概览

```text
仿真工作站启动(RViz2 + SNPApplication 面板)
  → 初始化
  → 批准扫描运动规划 / 加载扫描轨迹 / 执行扫描
  → 工业重建,生成 results_mesh.ply
  → 在 RViz 中用多边形工具圈选待加工区域 + 设置 Start Point
  → 生成工具路径(Tool Path)
  → 生成机器人打磨运动规划
  → 批准并执行抛光轨迹(仿真)
```

## 3. 环境要求

- Linux 桌面(本机为 `/home/liangfx/snp`)或 Windows + WSL 2
- Docker / Docker Compose,支持 X11 图形转发
- 镜像:`ghcr.io/ros-industrial-consortium/snp_automate_2023:jazzy-master`(hc10 原版镜像)
- 不需要在主机上安装 ROS(Humble 也不需要);ROS 2 Jazzy 在容器内

## 4. 启动与停止

### 4.1 首次启动(拉取/启动)

```bash
cd /home/liangfx/snp
chmod +x scripts/*.sh
./scripts/run_simulation.sh
```

脚本做的事:建运行目录 → `docker compose pull` → `docker compose up -d` →
打印容器状态与最近日志。容器名为 `snp_automate_2023_sim`。

### 4.2 日常重启(RViz 被关 / 改了配置想重新加载)

```bash
./scripts/restart_demo.sh
```

### 4.3 检查容器

```bash
docker ps --filter name=snp_automate_2023_sim
docker exec snp_automate_2023_sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 node list'
```

正常能看到:`/rviz`、`/reconstruction_sim_node`、`/tool_path_planning_server`、
`/snp_planning_server`、`/motoros2_simulator`、`/motion_execution_simulator`。

### 4.4 停止 / 删除

```bash
docker compose -f docker/compose.sim.yml down            # 停,保留镜像
docker rm -f snp_automate_2023_sim                       # 删容器
# 镜像占用很大,确不再用再删:
# docker image rm ghcr.io/ros-industrial-consortium/snp_automate_2023:jazzy-master
```

## 5. SNPApplication 操作流程(面板逐按钮)

RViz 打开后,SNPApplication 流程面板集成在界面中。按顺序操作(详细说明见
`PROJECT_WORKFLOW_CN.md` 第 7 节):

**阶段 A 初始化**:点击初始化/开始流程,等待服务和关节状态就绪。

**阶段 B 扫描与重建**:依次批准

```text
Approve Scan Motion Plan Creation
Load Trajectory From File      # 使用 config/scan_traj.yaml
Approve Scan Execution
```

期望日志:两次按钮后出现

```text
Start Reconstruction -> SUCCESS
Execute Scan Motion    -> SUCCESS
Stop Reconstruction    -> SUCCESS
```

完成后检查重建网格:

```bash
ls -lh runtime/snp_home/snp/meshes/results_mesh.ply
```

> 提示:原版扫描轨迹的关节名是 hc10 命名 `joint_1_s … joint_6_t`。
> 如果点太快、关节状态还没发布,可能报 `Get Current Joint State -> FAILED`,
> 等 1–2 秒再重新批准当前步骤即可。

**阶段 C 圈选加工区域**:在 RViz 里选 Polygon 多边形工具,在重建出的工件表面
**圈选一块较小、连续的区域**;然后必须用 TPP 工具单独**设置 Start Point(起点)**。
只画区域不设起点会报 `Start point has not been set`。

**阶段 D 生成工具路径**:点击规划,期望:

```text
Plan Tool Paths -> SUCCESS
```

若报 `Tool paths are empty!`/`Start point has not been set`,清除重画一个更小的
连续区域并重新设置起点。

**阶段 E 运动规划**:依次批准,期望:

```text
Approve Motion Plan Generation  -> SUCCESS
GenerateMotionPlanService       -> SUCCESS
Combine Trajectories            -> SUCCESS
```

**阶段 F 执行打磨**:批准 `Approve Process Motion Execution`,仿真执行器接受轨迹
并返回 SUCCESS(默认简化执行模式只验证流程,不播放完整关节动画;如需 RViz 实体
模型按轨迹运动,见第 7 节 ROS 2 Control 模式)。

## 6. 机器人执行模式

### 6.1 默认:简化执行(execution_simulator)

`docker/compose.sim.yml` 默认 `SNP_BYPASS_EXECUTION: "true"`,适合快速验证扫描、
重建、TPP、运动规划整条流程。日志出现 SUCCESS 不代表机器人已逐点播放轨迹。

### 6.2 ROS 2 Control 模拟硬件模式(能看到机器人动)

```bash
./scripts/run_ros2_control_simulation.sh
```

此模式用 `docker/compose.ros2_control.override.yml` 把 `SNP_BYPASS_EXECUTION` 置为
`"false"`,启动 `ros2_control_node` + `joint_trajectory_position_controller` +
`joint_state_broadcaster`(`mock_components/GenericSystem`),RViz 中实体会沿规划轨迹
运动。仍是虚拟仿真,不连真实 hc10。

## 7. 结果查看

- RViz:查看机器人、重建网格、选区、工具路径、规划轨迹;
- Blender:打开 `runtime/results_mesh_view.blend`,或 `File → Import → Stanford(.ply)`
  导入 `runtime/snp_home/snp/meshes/results_mesh.ply`。

## 8. 真机(hc10)运行(仅了解,本仓库不连真机)

原版项目支持真机,方式见 `ORIGINAL_README.md`:

1. 机器人控制柜安装 **MotoROS2**,版本需与 `dependencies.repos` 中的
   `motoros2_interfaces` 一致;
2. 相机手眼标定:`docker compose -f docker/calibration.docker-compose.yml`(真机标定容器),
   移动机械臂到 10–15 个位姿观察标定板,用 RViz 标定面板采集并保存;
3. 标定结果覆盖 `config/calibration.yaml`;
4. 真机启动:`cd docker && docker compose up`(注意设置 `CURRENT_UID` 与
   `ROS_DOMAIN_ID`)。

## 9. 常见问题速查

| 现象 | 处理 |
|---|---|
| 关节状态未发布导致 `Get Current Joint State -> FAILED` | 等 1–2 秒重新批准当前步骤 |
| `Tool paths are empty!` | 重画更小连续选区,并设 Start Point |
| `Start point has not been set` | 画完多边形后需单独设起点 |
| 服务 unreachable | `docker logs --tail=100 snp_automate_2023_sim` 查看,必要时 `restart_demo.sh` |
| RViz 卡顿 / 多个窗口 | 先 `docker rm -f snp_automate_2023_sim`,再 `restart_demo.sh`,只保留一个实例 |

更多见 [`docs/TROUBLESHOOTING_CN.md`](TROUBLESHOOTING_CN.md)。
