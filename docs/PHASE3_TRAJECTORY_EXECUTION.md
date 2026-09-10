# 阶段3：SNP离线轨迹生成 + eco65真机执行闭环

> **状态**：2026-09-10 实现完成  
> **目标**：打通「SNP容器算轨迹 → eco65真机执行」最小闭环  
> **前置**：阶段2（eco65原生真机实动✅）

---

## 架构与流程

```
[SNP 离线轨迹生成]
  mesh / noether 工具路径 → motion planning
  ↓ (导出)
[轨迹YAML格式]
  joint_names + trajectory points (6关节角度时间序列)
  ↓ (转换)
[FollowJointTrajectory ROS消息]
  (JointTrajectory)
  ↓ (执行)
[真机 rm_group_controller]
  UDP → 真机电机 → 打磨运动
```

---

## 实现组件

### 1. 轨迹生成：`scripts/generate_trajectory.py`

**功能**：离线生成打磨轨迹

**输入**：
- mesh 文件路径（可选，当前版本用测试轨迹）
- noether 配置（可选）

**输出**：`runtime/snp_home/test_trajectory.yaml`

**YAML 格式**：
```yaml
joint_names: [joint1, ..., joint6]
points:
  - positions: [j1, j2, j3, j4, j5, j6]      # 关节角度（弧度）
    velocities: [...]                        # 速度（可选）
    accelerations: [...]                     # 加速度（可选）
    time_from_start: 3.0                     # 时间戳（秒）
  - ...
```

**用法**：
```bash
cd /home/liangfx/snp
python3 scripts/generate_trajectory.py [output_path]
# 默认输出: runtime/snp_home/test_trajectory.yaml
```

### 2. 轨迹格式转换：`scripts/convert_trajectory_to_fjt.py`

**功能**：YAML → ROS JointTrajectory 消息 → 真机执行

**支持两种模式**：

#### 模式A：格式验证（本地）
```bash
python3 scripts/convert_trajectory_to_fjt.py /path/to/trajectory.yaml
```
输出轨迹信息、关键点检查。

#### 模式B：真机执行
```bash
export ROS_DOMAIN_ID=10
python3 scripts/convert_trajectory_to_fjt.py /path/to/trajectory.yaml --execute
```
需要：
- ROS 2 环境 + rclpy
- 真机驱动 `rm_driver` + `rm_control` 已启动
- rm_group_controller 监听 FJT action

---

## 使用流程（Phase 3.3 验证）

### 前置条件

1. **真机准备**
   ```bash
   # 宿主启动真机驱动
   ./scripts/start_real.sh
   # 验收：/joint_states 刷新、RViz 显示真机模型
   ```

2. **轨迹生成**
   ```bash
   # 在宿主生成轨迹YAML
   python3 scripts/generate_trajectory.py
   # 输出: runtime/snp_home/test_trajectory.yaml (7个关键点，14s)
   ```

3. **轨迹验证**
   ```bash
   # 检查格式和关键点
   python3 scripts/convert_trajectory_to_fjt.py runtime/snp_home/test_trajectory.yaml
   ```

4. **真机执行（端到端验证）**
   ```bash
   export ROS_DOMAIN_ID=10
   python3 scripts/convert_trajectory_to_fjt.py runtime/snp_home/test_trajectory.yaml --execute
   ```

   期望输出：
   ```
   等待真机 FollowJointTrajectory 服务...
   发送轨迹到真机: 7 个关键点, 总时长 14s
   目标已接受，等待执行...
   ✅ 轨迹执行完成: ...
   ```

### 真机运动验证

运动过程（14秒）：
1. **0-3s**：归位 → 靠近工件上方
2. **3-5s**：从0.8→0.7关节角（下降接触）
3. **5-9s**：打磨往返（关节4 ±0.1弧度摆动）
4. **9-11s**：撤离上升
5. **11-14s**：归位

**可视化**：
- RViz 模型实时跟随真机关节（FJT feedback）
- 真机末端应呈现典型的打磨运动（平面往返 + 竖向进给）

---

## 后续扩展

### Phase 3.1 高级：noether 集成

替换测试轨迹，真正对接 noether 工具路径：

```python
# generate_trajectory.py 改进
from noether_planner import ToolPathPlanner

def generate_from_mesh(mesh_path: str, config: Dict) -> JointTrajectory:
    """
    从扫描mesh生成打磨轨迹
    1. noether 生成工具路径（接触点序列）
    2. IK 规划 → 关节轨迹
    3. 路径平滑 → JointTrajectory
    """
    planner = ToolPathPlanner(config)
    contact_points = planner.generate_tool_path(mesh_path)
    joint_traj = plan_joint_trajectory(contact_points)
    return joint_traj
```

### Phase 3.2 高级：轨迹优化

- 关节加速度/速度限制
- 碰撞检测（URDF + tesseract）
- 平滑度优化（5阶多项式插值）

### Phase 4：完整闭环

- **扫描端**：真相机实时扫描 → mesh 更新
- **规划端**：增量规划 → 轨迹更新
- **执行端**：流式发送关键点给真机（如需流式）

---

## 已知限制

1. **当前轨迹**：测试轨迹（硬编码关键点）
   - 后续必须集成 noether / motion_planning 模块
   
2. **真机可达性**：RM-ECO65 近侧半圆可达，超出部分无法打磨
   - 需 mesh 预检查

3. **轨迹执行**：整段 FJT action（无流式点队列）
   - RM 驱动原生支持，推荐保持

4. **SNP 规划服务**：真机URDF下无法启动（编译层问题）
   - 绕过方案：离线生成 → 导出 → 执行（当前方案）

---

## 提交记录

- **bd13d2d** (2026-09-10): `feat(phase3): 轨迹生成+转换脚本 — 离线生成→eco65执行最小闭环`
  - `scripts/generate_trajectory.py`：轨迹离线生成（测试版）
  - `scripts/convert_trajectory_to_fjt.py`：YAML→FJT转换+真机执行

---

## 下一步（阶段4）

[ ] 集成 noether / motion_planning 到轨迹生成脚本  
[ ] 真机完整打磨工件验证（扫描→规划→执行）  
[ ] 可靠性测试与轨迹优化  
