#!/usr/bin/env python3
"""直接给 rm_group_controller 发一段 FJT 轨迹（宿主 DOMAIN_ID=10）。

绕过 SNP BT，直接验证/驱动真机，或把真机归位到指定关节角。

用法：
  export ROS_DOMAIN_ID=10
  python3 scripts/send_fjt.py                      # 归零 [0]*6
  python3 scripts/send_fjt.py 0.5 -0.3 0.9 0 0 0   # 指定 6 关节弧度
"""
import sys
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import builtin_interfaces

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ACTION = "/rm_group_controller/follow_joint_trajectory"
DURATION_S = 5.0  # 运动到位的总时长（秒），可调

def main():
    if len(sys.argv) == 1:
        target = [0.0] * 6
    elif len(sys.argv) == 7:
        target = [float(a) for a in sys.argv[1:7]]
    else:
        print(__doc__)
        sys.exit(1)

    rclpy.init()
    node = Node("fjt_zero")
    cli = node.create_client(FollowJointTrajectory, ACTION)
    node.get_logger().info(f"等待 action 服务 {ACTION} ...")
    while not cli.wait_for_service(timeout_sec=1.0):
        pass
    node.get_logger().info("服务在线，发送归位轨迹 → " + ", ".join(f"{j}={v:.3f}" for j, v in zip(JOINTS, target)))

    pt = JointTrajectoryPoint()
    pt.positions = target
    pt.time_from_start = builtin_interfaces.msg.Duration(sec=DURATION_S)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [pt]

    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if gh is None:
        node.get_logger().error("goal rejected")
        sys.exit(1)

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut)
    result = res_fut.result()
    code = result.result.error_code if result else -1
    node.get_logger().info(f"执行完成 error_code={code}")
    rclpy.shutdown()
    sys.exit(0 if code == 0 else 2)

if __name__ == "__main__":
    main()
