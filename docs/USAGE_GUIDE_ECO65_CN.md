# eco65(睿尔曼)打磨仿真使用说明

> 适用对象:本仓库**把机械臂从 Motoman hc10 替换为睿尔曼 eco65(rm_eco65)之后**
> 的版本(即当前工作区的配置,相对 git HEAD 有未提交改动)。
> 目标:在仿真里用 eco65 跑通打磨流程,并保持 ros2_rm_robot 代码栈兼容,便于后续接真实 eco65。

## 1. 与 hc10 原版的差异(先看这个)

| 项目 | hc10 原版(git HEAD) | 本版 eco65(当前工作区) |
|---|---|---|
| 机器人 | Motoman HC10 | 睿尔曼 eco65(rm_eco65) |
| URDF 根节点 / 名称 | `motoman_hc10` | `rm_eco65_workcell`(baselink 为根) |
| 坐标系参考 | `base_link` | `baselink` |
| 关节名 | `joint_1_s` … `joint_6_t` | `joint1` … `joint6` |
| 运动链(IK 用) | `base_link → tool0` | `baselink → Link6` |
| 安装(相对 table) | hc10 自带 | `table_to_base` 固定关节 `xyz="-0.61 0 0.723"` |
| 法兰/相机挂点 | `flange` | `Link6`(flange→`motoros2/r1/flange`) |
| 工具 TCP | `sand_tcp`(继承 hc10 安装位姿) | `sand_tcp`(同前,挂 Link6) |
| 仿真镜像 | 官方 `ghcr.io/…/snp_automate_2023:jazzy-master` | 本地自定义镜像(见 §3) |

> 注意:eco65 臂长比 hc10 短很多,**工件模型 `meshes/part_scan.ply` 已整体沿 x 平移
> dx=-0.32**(让工件进入 eco65 可达范围)。原始工件备份在
> `/tmp/part_scan_backup.ply`,如需还原可拷回。

## 2. 软件工作流(和 hc10 版一致)

```text
RViz2 + SNPApplication
  → Initialize
  → 扫描运动规划 / 执行扫描(eco65 关节空间轨迹)
  → 工业重建生成 results_mesh.ply
  → 圈选 ROI + 设置 Start Point
  → Plan Tool Paths
  → Generate Motion Plan
  → 批准并执行打磨轨迹
```

## 3. 环境与镜像(重要:不再是官方镜像)

本版仿真容器 `snp_automate_2023_sim` 由**本地 `docker/Dockerfile.custom`** 构建:

```dockerfile
FROM ghcr.io/ros-industrial-consortium/snp_automate_2023:jazzy-master
# 把宿主 /home/liangfx/ros2_ws/src/ros2_rm_robot 下的
# rm_description / rm_control / rm_moveit2_config / rm_ros_interfaces 拷进镜像
# 并 colcon build --packages-select ... --symlink-install
```

因此 eco65 的 URDF(在 `urdf/rm_description/`)来自宿主 `ros2_ws` 的 rm_robot 代码栈,
**改这些源文件后需要重新 build 镜像**。

## 4. 启动与重启

### 4.1 首次(构建镜像 + 启动)

```bash
cd /home/liangfx/snp
docker compose -f docker/compose.sim.yml build        # 构建含 eco65 rm 包的镜像
./scripts/restart_demo.sh                             # 删旧容器 + compose up
```

当前镜像名:`snp-automate-2023-polishing-simulation-snp_automate_2023_sim`(约 13.8 GB,
已在 `docker images` 中,一般无需重建)。

### 4.2 日常重启(改了 config/urdf/meshes 想重新加载)

```bash
./scripts/restart_demo.sh
```

> `scripts/restart_demo.sh` 会 `docker rm -f snp_automate_2023_sim` 再
> `docker compose up -d`,并把宿主机 `/home/liangfx/snp/{config,launch,urdf,meshes}`
> 实时挂载进容器,所以 **config/launch/urdf/meshes 改动无需重编镜像**,重启容器即生效。

### 4.3 查看容器 / 节点

```bash
docker ps --filter name=snp_automate_2023_sim
docker exec snp_automate_2023_sim bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 node list'
docker logs --tail=100 snp_automate_2023_sim
```

## 5. 关键配置(eco65 版改了这些)

| 文件 | 作用 |
|---|---|
| `urdf/workcell.xacro` | 引入 eco65 模型;`table_to_base` 把 baselink 放桌面上 |
| `urdf/rm_description/rm_eco65.urdf.xacro` | eco65 本体(来自 rm_description) |
| `launch/start.launch.xml` | `reference_frame=baselink`、`target_mount_frame=baselink`、`scan_disabled_contact_links=[table,baselink,floor]`、home 全零、`controller_joint_names=joint1..6` |
| `config/workcell_plugins.yaml` | IK 插件 `base_link: baselink` / `tip_link: Link6` |
| `config/workcell.srdf` | robot `rm_eco65_workcell`,chain `baselink→Link6` |
| `config/scan_traj.yaml` | eco65 关节空间的 14 点扫描轨迹(首尾归零) |
| `config/controllers.yaml` | joints=joint1..6 |
| `motoros2/motoros2_config.yaml` | joints=joint1..6(仿真用) |
| `meshes/part_scan.ply` | 已平移 dx=-0.32 的工件模型(仿真重建的"真相"来源) |

## 6. SNPApplication 操作流程(逐按钮)

与 hc10 版完全相同的面板,只是机器人换成 eco65:

1. **Initialize**:点击初始化,等 `Initialize Flags -> SUCCESS`;
2. **批准扫描**:`Approve Scan Motion Plan Creation` → 机械臂走一段 eco65 扫描轨迹
   并回到原位;
3. **重建**:批准扫描执行后,`Start Reconstruction -> SUCCESS` /
   `Stop Reconstruction -> SUCCESS`(第一次跑若报
   `Permission denied: …/results_mesh.ply`,把旧文件删掉再跑,见 §8);
4. **圈 ROI + 设 Start Point**:RViz 里用 Polygon 工具在工件表面框选一个
   **较小、连续**的区域,再用 TPP 工具设置起点;
5. **Plan Tool Paths**:期望 `Plan Tool Paths -> SUCCESS`(工具路径数量可在
   `/tool_paths` 话题看到);
6. **Generate Motion Plan**:期望 `GenerateMotionPlanService -> SUCCESS`。

## 7. 当前状态(截至 2026-09-04)

- ✅ eco65 模型正常加载,TF 树 `baselink→Link1..Link6` 完整,末端 frame
  (Link6 / camera_frame / sand_tcp)与数值 FK 毫米级吻合(用 `scripts/eco65_fk.py` 验证);
- ✅ 扫描轨迹正确执行(不再钻到桌子下方);
- ✅ 重建后工件正常出现;圈选 ROI + Plan Tool Paths 成功;
- ✅ **靠机械臂一侧半圆区域已能规划打磨**(2026-09-04 下午):在该区域框选 ROI 后
  `GenerateMotionPlanService -> SUCCESS`(求解器 `OPT_CONVERGED`、无碰撞、样条时间
  参数化成功)。说明 eco65 的 IK / 运动规划链路本身正常——此前整条 tool path 位姿
  IK 0/47 **不是求解器假阴性**,而是路径里含大倾角贴附的点超可达。姿态可达性是
  **区域相关**的。
- ⚠️ **剩余区域姿态不可达**:需沿 mesh 法线(~32°)大倾角压磨的面仍 `IK 0/47`。
  当前底座朝向 + 工具安装位姿(`sand_tcp` 安装 RPY 继承自 hc10 的
  `rpy=(0,-90°,-135°)`)达不到这些姿态。**下一步方向**:
  1. 先把已可达半圆区域的「执行打磨」跑通看效果;
  2. 重新设计 `sand_tcp_joint` 安装 RPY,或给底座加俯仰/偏航旋转找可达摆放;
  3. 找到可达布局后整面重测运动规划。

> 排查脚本:`scripts/eco65_ik.py`(位置 IK)、`scripts/eco65_toolpath_analysis.py`
> (整条 tool path 位姿可达性分析)。改动后请重启容器并在 RViz 重新框 ROI。

## 8. 常见问题(eco65 版)

| 现象 | 处理 |
|---|---|
| `Stop Reconstruction -> FAILED  Permission denied: results_mesh.ply` | 容器内用户 uid=1006 无权限写旧文件;删掉 `runtime/snp_home/snp/meshes/results_mesh.ply` 再跑,让容器新建 |
| `Plan Tool Paths -> FAILED  Tool paths are empty` | 重新框选**较小连续**的 ROI,并设置 Start Point(不是 eco65 bug,是交互要求) |
| `GenerateMotionPlanService -> FAILED` | 工具路径位姿超出 eco65 可达范围,见 §7 排查方向 |
| TF 里找不到 `base_link` | 正常:eco65 根是 `baselink`,不是 `base_link` |
| 改了 urdf/config 不生效 | 用 `scripts/restart_demo.sh` 重启容器(挂载实时生效);改了 `ros2_ws` 里 rm 源码才需要重 build 镜像 |

## 9. 相关文件

- 使用说明(本文件):`docs/USAGE_GUIDE_ECO65_CN.md`
- 项目进度:`docs/PROGRESS_2026-09-03.md`
- 完整原版流程参考:`docs/PROJECT_WORKFLOW_CN.md`、`docs/RUN_GUIDE_CN.md`
- hc10 原版使用说明:`docs/USAGE_GUIDE_ORIGINAL_HC10_CN.md`
