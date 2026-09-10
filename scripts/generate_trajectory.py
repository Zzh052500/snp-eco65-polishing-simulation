#!/usr/bin/env python3
"""
Phase 3.1 - 生成打磨轨迹并导出

离线生成一条简单的打磨轨迹：
- 输入：mesh + noether配置（或直接指定起止位置）
- 输出：轨迹YAML（JointTrajectory格式）

当前版本：测试轨迹（后续对接noether）
"""

import yaml
import sys
from typing import List, Dict, Any
from dataclasses import dataclass

@dataclass
class JointTrajectoryPoint:
    positions: List[float]
    velocities: List[float] = None
    accelerations: List[float] = None
    time_from_start: float = 0.0

@dataclass
class JointTrajectory:
    joint_names: List[str] = None
    points: List[JointTrajectoryPoint] = None

    def __post_init__(self):
        if self.joint_names is None:
            self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        if self.points is None:
            self.points = []

def create_test_polishing_trajectory() -> JointTrajectory:
    """
    生成测试打磨轨迹

    简单场景：
    1. 归位（0, 0, 0, 0, 0, 0）
    2. 靠近工件（上方接近）
    3. 打磨运动（平面往返）
    4. 撤离
    5. 归位
    """
    traj = JointTrajectory()

    # 关键点定义（后续从noether导入）
    keypoints = [
        # [j1, j2, j3, j4, j5, j6], time_s
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0),           # 0s: 归位
        ([0.3, -0.5, 0.8, 0.0, 0.0, 0.0], 3.0),         # 3s: 靠近工件
        ([0.3, -0.5, 0.7, 0.0, 0.0, 0.0], 5.0),         # 5s: 接触工件
        ([0.3, -0.5, 0.7, 0.1, 0.0, 0.0], 7.0),         # 7s: 打磨1
        ([0.3, -0.5, 0.7, -0.1, 0.0, 0.0], 9.0),        # 9s: 打磨2（往返）
        ([0.3, -0.5, 0.8, 0.0, 0.0, 0.0], 11.0),        # 11s: 撤离
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 14.0),         # 14s: 归位
    ]

    for positions, time_s in keypoints:
        pt = JointTrajectoryPoint(
            positions=positions,
            velocities=[0.0] * 6,
            accelerations=[0.0] * 6,
            time_from_start=time_s
        )
        traj.points.append(pt)

    return traj

def trajectory_to_dict(traj: JointTrajectory) -> Dict[str, Any]:
    """转换为可序列化的字典"""
    return {
        'joint_names': traj.joint_names,
        'points': [
            {
                'positions': pt.positions,
                'velocities': pt.velocities or [0.0] * len(traj.joint_names),
                'accelerations': pt.accelerations or [0.0] * len(traj.joint_names),
                'time_from_start': pt.time_from_start,
            }
            for pt in traj.points
        ]
    }

def save_trajectory_yaml(traj: JointTrajectory, output_path: str):
    """保存轨迹为YAML"""
    data = trajectory_to_dict(traj)
    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"✅ 轨迹已导出: {output_path}")

def main():
    if len(sys.argv) < 2:
        output_path = "/home/liangfx/snp/runtime/snp_home/test_trajectory.yaml"
    else:
        output_path = sys.argv[1]

    print(f"生成测试打磨轨迹...")
    traj = create_test_polishing_trajectory()
    print(f"  关键点数: {len(traj.points)}")
    print(f"  总时长: {traj.points[-1].time_from_start}s")

    save_trajectory_yaml(traj, output_path)
    print(f"\n轨迹结构:")
    print(f"  关节: {', '.join(traj.joint_names)}")
    print(f"  点数: {len(traj.points)}")
    for i, pt in enumerate(traj.points):
        print(f"    [{i}] t={pt.time_from_start:5.1f}s: {[f'{x:.2f}' for x in pt.positions]}")

if __name__ == "__main__":
    main()
