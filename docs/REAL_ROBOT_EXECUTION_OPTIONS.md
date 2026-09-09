# RM-ECO65 真机执行链路 —— 方案评估（2026-09-09）

## 一、问题本质

SNP_Automate_2023 整套是为 **Motoman 机器人 + MotoROS2** 设计的。真机换成 **RM-ECO65**（rm_driver 用 UDP 协议，rm_control 提供 `rm_group_controller/follow_joint_trajectory` action），出现 motos2 依赖不匹配：

- RViz/BT 的 **Enable Robot** 调 `start_point_queue_mode` 服务 —— motos2 才有
- RViz/BT 的 **Disable Robot** 调 `stop_traj_mode` 服务 —— motos2 才有
- 容器里 `fjt_proxy`（motoros2_fjt_pt_queue_proxy）等 MotoROS2 的 `queue_traj_point` server —— RM 没有
- RViz 里 [Get Current Joint State] FAILED / [Enable Robot] FAILED 都源于此

## 二、关键澄清：真正"动"靠什么？

读 BT（`config/snp_automate.xml`）确认：

```
扫描接近流程（process_motion 等）:
  1. GetCurrentJointState     → 读 /joint_states
  2. UpdateTrajectoryStartState → 更新轨迹起点
  3. Enable Robot (StartPointQueueMode) → 调 start_point_queue_mode  ← 仅"使能开关"
  4. Execute Approach Motion (FollowJointTrajectoryAction)
                              → action_name = {follow_joint_trajectory_action}  ← 真正运动
  5. Disable Robot (Trigger)  → 调 stop_traj_mode                          ← 仅"下使能"
```

**核心结论**：
- **真正驱动机械臂运动的是 FJT action**，BT 已把它指向 `rm_group_controller/follow_joint_trajectory`（真机分支正确）
- **Enable/Disable Robot 只是两个状态开关服务**，RM 真机缺这两个服务导致 RViz 报错
- **fjt_proxy 不是执行必需**——它只服务 motos2 点队列，RM 场景可绕开

## 三、方案对比

### 方案 A：补两个"使能开关"服务（最小侵入，推荐）

写一个小节点（如 `rm_robot_service_proxy`）提供：
- `start_point_queue_mode` 服务 → 返回 READY（可选：真正给 RM 上使能）
- `stop_traj_mode` 服务 → 返回 success（可选：真正下使能）

**让 BT/RViz 的 Enable/Disable Robot 通过**，FJT 部分已指向 rm_group_controller。

**优点**：
- 改动最小，1 个新节点，不碰 BT 架构
- RM 实际使能（若需要）可在回调里接 rm SDK
- 仿真完全不受影响（仿真的 motoros2_simulator 照旧）
- RViz 报错消失，能进到真正的 FJT 执行

**缺点**：
- 需要一个"假"服务，语义上 Enable Robot 对 RM 不是真点队列模式（但不影响，因为 RM 不走点队列）

**工作量**：约 50-100 行 Python，放 launch 真机分支。

---

### 方案 B：复用 fjt_proxy，做一个 RM 的 queue server 适配层

让 fjt_proxy 正常初始化——写一个节点实现 `queue_traj_point` server，把每个轨迹点转成 RM 的 FJT 或 MoveJ 下发。

**优点**：
- 完全对齐 motos2 架构（fjt_proxy 能初始化）
- 支持逐点流式执行（连续轨迹时平滑）

**缺点**：
- **复杂**：要实现 QueueTrajPoint 完整协议（BUSY/OK 状态机）、点缓冲、与 rm_group_controller 的同步
- RM 驱动本身接受整段 FJT trajectory，逐点喂反而多余
- fjt_proxy 自带 CPU leak（README 明说 WIP）
- 工作量大、收益低

**工作量**：300+ 行，且要处理状态机/同步，风险高。

---

### 方案 C：完全绕开 BT 的 Enable Robot，直接验证 FJT

不做使能服务，直接用命令行发一个简单 FJT trajectory 给 `rm_group_controller`，先验证"真机能不能按轨迹动"。

**优点**：
- 最快验证真机运动能力（阶段2 核心验收）
- 不依赖 BT/服务代理

**缺点**：
- 只验证运动，不验证完整 BT 打磨流程
- 后续跑 BT 仍需方案 A 的使能服务

**工作量**：写个 30 行 python 发 FJT goal。

---

## 四、推荐路线（组合拳）

1. **先做方案 C**（30 分钟）：命令行发一段简单 FJT 轨迹 → 确认真机能按轨迹动。这是阶段 2 的"真机运动"硬验收，必须先证。
2. **再做方案 A**（1 小时）：写 `rm_robot_service_proxy` 补两个使能服务 → 让 RViz/BT 的 Enable Robot 通过 → 完整跑 BT 打磨流程（阶段 3）。

方案 B 建议不做——RM 原生 FJT 已够，逐点喂无必要。

## 五、需要你确认的问题

1. RM 真机运动前**需不需要上使能/手闸**？若需要，Enable Robot 回调里要接 RM 的对应指令；若 RM 一直处于可运动状态，返回 READY 即可。
2. 你要先验证哪个？先 C（快证运动）还是直接 A（完整流程）？

---

## 六、最终决策（2026-09-09）：转 eco65 原生路线

方案 C（直接 FJT 验证真机运动）**已通过**，且比预期走得更远——**eco65 自带 MoveIt 已能完整拖拽规划并驱动真机**（阶段2联动验收，见 memory）。随后尝试在 SNP RViz 里跑完整 BT 主流程（方案 A 补 Enable/Disable 服务已让门控通过），但被 3 个**编译层问题**永久堵死（均无源码，见 memory 转折一节）：

- `spin_some() called while already spinning`（BT 在 RViz 内 executor 重入）
- snp_motion_planning_node "Failed to parse URDF / configure environment" + tesseract 插件 "Bad file descriptor"
- GetCurrentJointState / UpdateTrajectoryStartState 只有二进制，无源码可改

**结论**：真机执行骨干放弃 SNP 自带 BT，改用 eco65 原生 MoveIt（已证实能驱动真机）；SNP 保留做离线规划/打磨轨迹生成。相关工具：`scripts/start_real_moveit.sh`（一键复现阶段2）、`scripts/send_fjt.py`（直发 FJT）。

本文件的方案 A/B 评估仍可作为参考，但**不再作为真机 BT 执行的主力路线**——除非未来拿到 SNP 源码、能在真机 URDF 上修好 planning server 与 BT executor，才有回头价值。
