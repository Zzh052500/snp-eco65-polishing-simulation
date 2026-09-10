#!/usr/bin/env python3
"""
Phase 3.2 - 轨迹格式转换与执行

输入：SNP导出的轨迹YAML
输出：eco65 FollowJointTrajectory action 可执行格式

支持：
  1. 加载YAML轨迹
  2. 转换为JointTrajectory消息格式
  3. 可选：直接发送到真机执行（需ROS_DOMAIN_ID=10）
"""

import sys
import yaml
import time
from typing import List, Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

def load_trajectory_yaml(yaml_path: str) -> Dict[str, Any]:
    """加载YAML轨迹文件"""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data

def yaml_to_joint_trajectory_msg(traj_data: Dict[str, Any]) -> 'JointTrajectory':
    """转换YAML到ROS JointTrajectory消息"""
    if not HAS_ROS:
        raise ImportError("需要ROS 2环境（rclpy）")

    msg = JointTrajectory()
    msg.joint_names = traj_data['joint_names']

    for pt_data in traj_data['points']:
        pt = JointTrajectoryPoint()
        pt.positions = pt_data['positions']
        pt.velocities = pt_data.get('velocities', [0.0] * len(msg.joint_names))
        pt.accelerations = pt_data.get('accelerations', [0.0] * len(msg.joint_names))

        time_s = pt_data['time_from_start']
        pt.time_from_start = Duration(sec=int(time_s), nanosec=int((time_s % 1.0) * 1e9))

        msg.points.append(pt)

    return msg

def print_trajectory_info(traj_data: Dict[str, Any]):
    """打印轨迹信息"""
    print(f"\n📋 轨迹信息:")
    print(f"  关节: {', '.join(traj_data['joint_names'])}")
    print(f"  点数: {len(traj_data['points'])}")
    print(f"  总时长: {traj_data['points'][-1]['time_from_start']:.1f}s")
    print(f"\n  关键点:")
    for i, pt in enumerate(traj_data['points']):
        pos_str = ', '.join(f"{x:6.2f}" for x in pt['positions'])
        print(f"    [{i:2d}] t={pt['time_from_start']:6.1f}s: [{pos_str}]")

async def send_trajectory_to_real_robot(traj_msg: 'JointTrajectory') -> bool:
    """
    发送轨迹到真机（eco65 rm_group_controller）

    前提：
    - ROS_DOMAIN_ID=10
    - 真机驱动和rm_group_controller已启动
    """
    if not HAS_ROS:
        print("❌ 未找到ROS 2环境，跳过真机执行")
        return False

    rclpy.init()
    node = Node("trajectory_executor")
    cli = node.create_client(FollowJointTrajectory, "/rm_group_controller/follow_joint_trajectory")

    node.get_logger().info("等待真机 FollowJointTrajectory 服务...")
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("服务超时，真机不在线")
        rclpy.shutdown()
        return False

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj_msg

    node.get_logger().info(f"发送轨迹到真机: {len(traj_msg.points)} 个关键点, 总时长 {traj_msg.points[-1].time_from_start.sec}s")

    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)

    gh = fut.result()
    if gh is None:
        node.get_logger().error("目标被拒绝")
        rclpy.shutdown()
        return False

    node.get_logger().info("目标已接受，等待执行...")

    result_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)

    result = result_fut.result()
    if result is None:
        node.get_logger().error("执行被中止")
        rclpy.shutdown()
        return False

    node.get_logger().info(f"✅ 轨迹执行完成: {result}")
    rclpy.shutdown()
    return True

def main():
    if len(sys.argv) < 2:
        yaml_path = "/home/liangfx/snp/runtime/snp_home/test_trajectory.yaml"
    else:
        yaml_path = sys.argv[1]

    execute = '--execute' in sys.argv

    print(f"📂 加载轨迹: {yaml_path}")
    traj_data = load_trajectory_yaml(yaml_path)

    print_trajectory_info(traj_data)

    if execute:
        print(f"\n⚙️  转换为ROS消息格式并发送到真机...")
        try:
            traj_msg = yaml_to_joint_trajectory_msg(traj_data)
            import asyncio
            success = asyncio.run(send_trajectory_to_real_robot(traj_msg))
            sys.exit(0 if success else 1)
        except Exception as e:
            print(f"❌ 执行失败: {e}")
            sys.exit(1)
    else:
        print(f"\n✅ 轨迹格式验证成功")
        print(f"   执行: python3 {sys.argv[0]} {yaml_path} --execute  （需要ROS_DOMAIN_ID=10）")

if __name__ == "__main__":
    main()
