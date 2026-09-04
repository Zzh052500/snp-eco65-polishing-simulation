# SNP 打磨仿真 · RM-ECO65 版（独立项目）

本仓库是把 **SNP Automate 2023 打磨仿真**中的机械臂从 Motoman HC10 **替换为睿尔曼 RM-ECO65（`rm_eco65`）** 之后的独立版本。设计上**沿用 `ros2_rm_robot` 代码栈**（eco65 自身的 ROS 原生栈），让仿真结果能够平滑地**迁移到真实 eco65 机器人**上运行。

> 派生于 `ros-industrial-consortium/snp_automate_2023` 仿真 Demo，保留其许可证与源码，在此之上以 eco65 为目标做了整体改造。

## 📚 快速导航（先读这两篇）

| 文档 | 内容 |
|---|---|
| [`docs/PROGRESS_2026-09-03.md`](docs/PROGRESS_2026-09-03.md) | **进度总记录（权威）**：已完成改动、已修问题、当前卡点与下一步计划 |
| [`docs/USAGE_GUIDE_ECO65_CN.md`](docs/USAGE_GUIDE_ECO65_CN.md) | **eco65 使用指南**：镜像构建、启动/重启、逐按钮操作流程、常见问题 |

其他参考：原版完整流程 [`docs/PROJECT_WORKFLOW_CN.md`](docs/PROJECT_WORKFLOW_CN.md)、运行指南 [`docs/RUN_GUIDE_CN.md`](docs/RUN_GUIDE_CN.md)、hc10 原版对照 [`docs/USAGE_GUIDE_ORIGINAL_HC10_CN.md`](docs/USAGE_GUIDE_ORIGINAL_HC10_CN.md)。

## 项目目标

1. 在仿真中用 eco65 跑通「扫描 → 重建 → 工具路径规划 → 运动规划 → 执行打磨」完整闭环；
2. 全程保持 `ros2_rm_robot` 兼容（方案 2），仿真结果可平移到真实 eco65；
3. 闭环后在真机上复现同一套打磨流程。

## 与 hc10 原版的差异

| 项目 | hc10 原版 | 本版 eco65 |
|---|---|---|
| 机器人 | Motoman HC10 | 睿尔曼 RM-ECO65（`rm_eco65`） |
| URDF 名称 / 根坐标 | `motoman_hc10` / `base_link` | `rm_eco65_workcell` / `baselink` |
| 关节名 | `joint_1_s` … `joint_6_t` | `joint1` … `joint6` |
| 运动链（IK） | `base_link → tool0` | `baselink → Link6` |
| 安装 | hc10 自带 | `table_to_base` 固定关节 `xyz="-0.61 0 0.723"` |
| 法兰 / 相机挂点 | `flange` | `Link6`（flange → `motoros2/r1/flange`） |
| 仿真镜像 | 官方 `ghcr.io/…:jazzy-master` | 本地自定义镜像（`docker/Dockerfile.custom`） |
| 工件模型 | 原位置 | `meshes/part_scan.ply` 整体沿 x 平移 **dx=-0.32**（eco65 臂长更短，移近才可达；原始备份 `/tmp/part_scan_backup.ply`） |

eco65 运动学：**`baselink` 为根**，6 个旋转关节 `baselink→Link1(j1)→…→Link6(j6)`；末端 frame（`tool0_to_ee` / `tool0_to_camera` / `sand_tcp`）挂在 Link6 上，`sand_tcp_joint` 安装位姿继承 hc10 的 `rpy=(0,-90°,-135°)`。

## 软件工作流（与 hc10 版一致的面板）

```text
RViz2 + SNPApplication
  → Initialize
  → 批准扫描运动规划 / 执行扫描（eco65 关节空间轨迹）
  → 工业重建，生成 results_mesh.ply
  → 圈选 ROI + 设置 Start Point（TPP 工具）
  → Plan Tool Paths
  → Generate Motion Plan
  → 批准并执行打磨轨迹
```

## 环境与启动

eco65 仿真容器 `snp_automate_2023_sim` 由本地 `docker/Dockerfile.custom` 构建：以官方
`ghcr.io/ros-industrial-consortium/snp_automate_2023:jazzy-master` 为基础，把宿主
`/home/liangfx/ros2_ws/src/ros2_rm_robot` 下的 `rm_description / rm_control /
rm_moveit2_config / rm_ros_interfaces` 拷入并 `colcon build`。

**首次构建 + 启动：**

```bash
cd /home/liangfx/snp
docker compose -f docker/compose.sim.yml build   # 构建含 eco65 rm 包的镜像
./scripts/restart_demo.sh                          # 删旧容器 + compose up
```

**日常重启**（改了 `config/launch/urdf/meshes` 想重新加载）：

```bash
./scripts/restart_demo.sh
```

> `compose.sim.yml` 会把宿主 `/home/liangfx/snp/{config,launch,urdf,meshes}` **实时挂载**进容器，
> 所以这些目录的改动**无需重编镜像**，重启容器即生效；只有改了 `ros2_ws` 里的 rm 源码才需要重 build 镜像。

查看容器 / 节点：

```bash
docker ps --filter name=snp_automate_2023_sim
docker exec snp_automate_2023_sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 node list'
docker logs --tail=100 snp_automate_2023_sim
```

## 关键文件（eco65 版改动集中在这）

| 文件 | 作用 |
|---|---|
| `urdf/workcell.xacro` | 引入 eco65 自包含模型；`table_to_base` 固定 baselink 到桌面坐标 |
| `urdf/rm_description/rm_eco65.urdf.xacro` | eco65 本体模型（源自 rm_description，mesh 路径已改为本仓库 `meshes/`） |
| `launch/start.launch.xml` | `reference_frame / target_mount_frame = baselink`、`scan_disabled_contact_links=[table,baselink,floor]`、home 全零、`controller_joint_names=joint1..6` |
| `config/workcell_plugins.yaml` | IK 插件 `base_link: baselink` / `tip_link: Link6` |
| `config/workcell.srdf` | robot `rm_eco65_workcell`，chain `baselink→Link6` |
| `config/scan_traj.yaml` | eco65 关节空间 14 点扫描轨迹（首尾归零） |
| `config/controllers.yaml` / `motoros2/motoros2_config.yaml` | 关节 `joint1..6` |
| `meshes/part_scan.ply` | 平移 dx=-0.32 后的工件模型（重建的"真相"来源） |
| `docker/compose.sim.yml` + `docker/Dockerfile.custom` | eco65 自定义镜像与挂载配置 |
| `scripts/eco65_*.py` | FK / IK / 工具路径 / 摆放搜索等数值分析脚本 |

## 当前状态（2026-09-04 更新）

- ✅ eco65 模型正常加载；TF 树 `baselink→Link1..Link6` 完整，末端 frame 与数值 FK 毫米级吻合（`scripts/eco65_fk.py`）
- ✅ 扫描轨迹正确执行（不再钻到桌下）；重建后工件正常出现
- ✅ 圈选 ROI + Plan Tool Paths 成功；**靠机械臂一侧半圆区域 Generate Motion Plan 已通过**（求解器 `OPT_CONVERGED`、无碰撞、样条时间参数化成功）——确认 eco65 的 IK / 运动规划链路本身正常，此前整条 tool path 位姿 IK 0/47 **不是求解器假阴性**
- ⚠️ 其余区域**姿态可达性受限**：需沿 mesh 法线（~32° 大倾角）贴附的面，在现底座朝向 + `sand_tcp` 安装 RPY（继承自 hc10）下仍不可达

**下一步方向**（详见 `PROGRESS_2026-09-03.md` §三·1/§四/§五）：
1. 先把已可达半圆区域的「执行打磨」跑通，验证闭环最后一环；
2. 重新设计 `sand_tcp_joint` 安装 RPY，或给底座**加俯仰/偏航旋转**再搜可达摆放（纯平移不改变相对 baselink 的姿态需求），提升整面姿态可达率；
3. 找到能"整面带姿态打磨"的布局 → 改 `table_to_base` → 重启容器 → 重新框 ROI → 走通执行打磨，闭环整个流程。

## 目录结构

```text
config/        SNP / RViz / 工具路径规划参数（eco65 版）
launch/        ros2 launch 文件（start / test）
urdf/          workcell.xacro + rm_description/（eco65 模型 xacro）
meshes/        eco65 STL + 平移后的 part_scan.ply
scripts/       restart_demo.sh、eco65 FK/IK/工具路径分析脚本
docs/          PROGRESS、USAGE_GUIDE 等说明文档
docker/        compose.sim.yml + Dockerfile.custom（自定义镜像）
motoros2/      motoros2 配置（joint1-6）
runtime/       仿真运行产物（results_mesh.ply 等，不入库）
```

## 迁移到真机（规划）

仿真闭环跑通后，在宿主机 Humble 环境通过 `motoros2` 连接**真实 eco65**，
把「扫描 → 重建 → 工具路径 → 运动规划 → 打磨」流程整体平移到真机。由于本版本刻意保持
`ros2_rm_robot` 原生栈，仿真与真机之间无需更换控制器接口。
