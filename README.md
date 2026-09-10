# SNP · RM-ECO65 真机迁移 —— 阶段性总结

> **分支 `zhenjiqianyi` · 最新阶段记录 2026-09-10**（真机迁移第 1~3 天：09-08/09-09/09-10）
> 本 README 是**真机迁移全进度**的权威快照（阶段1-3已完成，阶段4+规划中）。
> 迁移前的仿真阶段（eco65 换臂打磨仿真闭环）见 [`docs/PROGRESS_2026-09-03.md`](docs/PROGRESS_2026-09-03.md) 与 [`docs/USAGE_GUIDE_ECO65_CN.md`](docs/USAGE_GUIDE_ECO65_CN.md)
> （旧版 README 内容已随 git 历史完整保留，最末提交 09-04）。
> 
> **完整文档索引**：[阶段3详细流程](docs/PHASE3_TRAJECTORY_EXECUTION.md) | [执行方案评估](docs/REAL_ROBOT_EXECUTION_OPTIONS.md) | [架构与决策记录](../../.claude/projects/-home-liangfx-Desktop/memory/snp-real-robot-migration.md)

把 SNP Automate 2023 打磨系统从 **Motoman + MotoROS2** 换到 **睿尔曼 RM-ECO65**，并把仿真流程搬到**真机**。核心判断贯穿全程：**绕开 SNP 自带 motoros2，用 eco65 自己的 driver/control 驱动真机。**

---

## 一、现状盘点（2026-09-09 硬验收结果）

| 目标 | 状态 |
|---|---|
| eco65 真机动（生态 MoveIt / 直连 FJT） | ✅ **已实机验证 SUCCEEDED**（阶段 2 硬验收过） |
| SNP Enable/Disable 门控服务 | ✅ 已打通（方案 A，CLI 实测 READY） |
| SNP 规划服务器在真机 URDF 下起来 | ❌ `Failed to parse URDF` / tesseract 插件加载失败 |
| SNP RViz BT 执行引擎在真机跑 | ❌ `spin_some` 重入崩，无 Go Home |

**根因性质**：SNP 整套（规划 + tesseract + RViz BT）是给 **motoman + 仿真工作台**（带 floor/工作台/打磨头/ros2_control 标签的 URDF）做的。真机换纯官方 URDF + 这些编译库在镜像里本来就是半残（插件 `Bad file descriptor`），想在真机上把 SNP 主流程完整跑起来，投入可能很大、且卡在看不到源码的地方 → **不再投入**。

**方向定案（用户认可）**：**"绕过 SNP 自带 motoros2、用 eco65 自己的 driver/control 做真机" → 成立**（机械臂动了）。真正稳的真机执行通道是 **eco65 原生链路**。SNP 只做离线规划 / 打磨轨迹生成。

---

## 二、这两天做了什么（09-08 → 09-09，逐提交）

### 📅 09-08 —— 真机迁移起步（`0ee9b7b`）
**域隔离 + 最小驱动 + 单机械臂**
- `docker/compose.real.yml`：真机容器（DOMAIN_ID=10，仿真保持 7，两域互不可见）
- `scripts/start_real.sh`：宿主 `rm_driver` + `rm_control` + 真机容器一键启动
- `launch/` 加 `sim_robot` 门控（`joint_state_publisher` 按真机/仿真开关）
- `urdf/.../rm_eco65.urdf.xacro` 小改 → 真机侧用**纯官方 eco65 模型**

### 📅 09-09 上午 —— 阶段 1 ✅ 真机 RViz 显示修复（`e140fc4`）
真机在 RViz 里显示的**始终是仿真环境**而非真机模型 → 根因：真机分支没切换 URDF。
- `launch/start.launch.xml`：`robot_description_file` 按 `sim_robot` 分支 → 真机加载 `workcell_real.xacro`（纯官方 rm_eco65.urdf，**无 floor/工作台/打磨头/ros2_control 标签**）
- `config/app_real.rviz`（新建）：真机专用 RViz，Fixed Frame=`baselink`（真机 URDF 无 floor），禁用 Open3D Mesh
- `docker/compose.real.yml`：加 `XDG_RUNTIME_DIR` 修 GUI 黑屏
- 修复 GUI 踩坑经验见 memory：`/tmp/runtime-1006` + `QT_QPA_PLATFORM=xcb`

### 📅 09-09 下午 —— 阶段 2 ✅ 真机实动 + 路线转折（`435f702` `f3c088b`）

**① eco65 原生 MoveIt 驱动真机实动（硬验收通过）**
生态自带栈直接上真机：eco65 RSP + `real_moveit_demo`（move_group + MoveIt RViz，`moveit_manage_controllers=false` → **复用已在跑的宿主 rm_control**，不重复起驱动）。RViz 拖末端 marker → **Plan & Execute → 真机真的动了**，日志铁证（`logs/eco65_moveit.log`）：
```
Plan and Execute request accepted
sending trajectory to rm_group_controller
rm_group_controller started execution / Goal request accepted!
Controller 'rm_group_controller' successfully finished
Completed trajectory execution with status SUCCEEDED
```
顺带验证**拖动示教反馈链**：真机手拖 → `/joint_states` → RSP → RViz 模型实时跟随。

**② 尝试 SNP RViz BT 主流程 → 被 3 个编译层问题堵死**
- 补 Enable/Disable 门控（方案 A：真机分支复用 `motoros2_simulator`，`StartPointQueueMode`→READY、`Trigger`→success，CLI 实测通过）
- 但继续推进即撞墙：RViz 点 Execute Motion Plan → `spin_some() called while already spinning` 重入崩；`GetCurrentJointState`/`UpdateTrajectoryStartState` 只有二进制无源码；无 Go Home。且 SNP 规划服务器（`snp_motion_planning_node`）在真机 URDF 下起不来（`Failed to parse URDF` / tesseract 插件 `Bad file descriptor`）

**③ 用户拍板：转 eco65 原生路线**
放弃把 SNP BT 主流程硬塞上真机；真机执行 = **eco65 原生**（已证会动），SNP 只做离线规划。固化产物见下表，根因与决策全量写入 memory 与 [`docs/REAL_ROBOT_EXECUTION_OPTIONS.md`](docs/REAL_ROBOT_EXECUTION_OPTIONS.md)（§六 最终决策）。

---

## 三、真机链路架构

```
[真机执行通道 = eco65 原生]
host  rm_driver (UDP 8089) + rm_control            # 提供 /rm_group_controller/follow_joint_trajectory (FJT)
   └─ eco65 MoveIt RViz(拖拽规划) → move_group → rm_group_controller FJT → rm_driver → 真机电机   ✅ SUCCEEDED
        (moveit_manage_controllers=false，复用宿主 rm_control，不重复起驱动)

[SNP 只做离线规划]   SNP 容器（DOMAIN 10 / sim_robot=false）
   扫 mesh → noether 工具路径 → 打磨轨迹          # 不与 eco65 MoveIt 同时起（避免 /robot_description、TF 双发布）
```

关键前提：真机控制器上电、网络通（TCP .18:8080 / UDP .95:8089）、急停在手；**真机实际在零位时**关节 `/joint_states` ≈ 全 0。

---

## 四、本阶段产物（工具 / 改动，均已入库）

| 文件 | 作用 |
|---|---|
| `scripts/start_real.sh` | 宿主 `rm_driver`+`rm_control` + SNP 真机容器启动（阶段 1 入门） |
| `scripts/start_real_moveit.sh` | **一键复现阶段 2**：检 FJT action → `docker stop` SNP 容器 → 起 eco65 RSP + MoveIt（`--stop` 反杀、不动驱动） |
| `scripts/send_fjt.py` | 绕过 BT 直发 FJT 给 `/rm_group_controller`（归位 / 指定 6 关节弧度），`ROS_DOMAIN_ID=10 python3 scripts/send_fjt.py [j1..j6]` |
| `launch/start.launch.xml` | 真机分支：`workcell_real.xacro` + `app_real.rviz` + `rm_group_controller` FJT + `motoros2_simulator`（Enable/Disable 门控） |
| `config/app_real.rviz` | 真机 RViz：Fixed Frame=`baselink`，禁 Open3D Mesh |
| `urdf/workcell_real.xacro` | 真机纯官方 rm_eco65.urdf（无工作台标签） |
| `docker/compose.real.yml` | 真机容器：DOMAIN 10、3 个 env var、XDG_RUNTIME_DIR |
| `docs/REAL_ROBOT_EXECUTION_OPTIONS.md` | 方案 A/B/C 评估 + §六 最终决策 / 根因定性 |

> 明确**不入库**：`src/`（RM 嵌套 git 仓库）、`runtime/**`（运行产物 results_mesh.ply 等）、`logs/`。`restart_demo.sh` 的 CRLF/权限噪音改动未纳入本阶段提交。

---

## 五、复现（下次真机测试直接照做）

```bash
# 1) 宿主驱动 + 真机容器（真机须已上电、零位）
./scripts/start_real.sh
#    验收：/joint_states 刷新（6 关节≈0）、RViz 显示真机模型

# 2) eco65 原生真机 MoveIt —— 阶段2 硬验收（复现实动）
./scripts/start_real_moveit.sh        # GUI 在 DISPLAY=:12.0
#    在 MoveIt RViz(Motion Planning, group=rm_group) 拖末端 marker → Plan & Execute → 真机动
#    停止：./scripts/start_real_moveit.sh --stop

# 3) 想绕开 GUI 直接发轨迹 / 归位
ROS_DOMAIN_ID=10 python3 scripts/send_fjt.py 0 0 0 0 0 0      # 归零
ROS_DOMAIN_ID=10 python3 scripts/send_fjt.py 0.5 -0.3 0.9 0 0 0  # 指定弧度
```

---

## 六、阶段 3 —— 离线轨迹生成 + 真机执行闭环（2026-09-10 ✅）

**核心路线**：不走 SNP RViz BT 主流程（真机URDF下无法启动），改为离线生成轨迹 + eco65原生执行。

**Phase 3.1 - 轨迹离线生成**（✅ bd13d2d）
- `scripts/generate_trajectory.py`：离线生成打磨轨迹 YAML
- 输出格式：`joint_names` + trajectory points（6 关节角度时间序列）

**Phase 3.2 - 轨迹格式转换**（✅ bd13d2d）
- `scripts/convert_trajectory_to_fjt.py`：YAML → ROS JointTrajectory 消息
- 支持模式：格式验证 / 真机直接执行（`--execute` + `ROS_DOMAIN_ID=10`）

**Phase 3.3 - 真机端到端验证**
使用流程：
```bash
# 1) 真机驱动启动
./scripts/start_real.sh

# 2) 离线生成轨迹
python3 scripts/generate_trajectory.py

# 3) 格式验证
python3 scripts/convert_trajectory_to_fjt.py runtime/snp_home/test_trajectory.yaml

# 4) 真机执行
export ROS_DOMAIN_ID=10
python3 scripts/convert_trajectory_to_fjt.py runtime/snp_home/test_trajectory.yaml --execute
```

详见 [`docs/PHASE3_TRAJECTORY_EXECUTION.md`](docs/PHASE3_TRAJECTORY_EXECUTION.md)。

---

## 七、下一步（阶段 4+）

1. **noether 集成**：真正从 mesh 生成工具路径，替换测试轨迹
2. **motion_planning 离线**：在 SNP 容器外离线规划，规避 tesseract 编译问题
3. **真相机扫描**：全流程打磨（扫描→规划→执行）；已知限制：超出近侧半圆不可达

---

*完整架构 / 踩坑 / 数据流见个人 memory：`snp-real-robot-migration.md`（~/.claude/projects/…/memory/）。推送仅限 `eco65-backup`（Zzh052500/snp-eco65-polishing-simulation），`origin` 为上游只读勿推。*
