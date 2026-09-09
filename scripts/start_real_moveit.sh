#!/usr/bin/env bash
# ============================================================================
# eco65 原生真机 MoveIt 启动脚本（阶段2 联动验收通过的路线，2026-09-09）
#
# 架构：真机执行走 eco65 自己的 driver/control/MoveIt，绕开 SNP 自带 motoros2。
#   MoveIt RViz(拖拽规划) → move_group → /rm_group_controller/follow_joint_trajectory
#   → rm_control(宿主) → rm_driver → 真机电机
#
# 前提：宿主 rm_driver + rm_control 已在跑（用 ./scripts/start_real.sh，或已手动起）。
#       真机控制器上电、急停在手。
#
# 用法：
#   ./scripts/start_real_moveit.sh          # 启动 eco65 原生 MoveIt 栈
#   ./scripts/start_real_moveit.sh --stop   # 停止 eco65 GUI 栈（宿主驱动不动）
#
# 注意：会先 docker stop snp_automate_2023_real，避免 /robot_description、TF
#       双发布与 eco65 MoveIt 冲突。要回 SNP 模式时 docker start 即可。
# ============================================================================
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$ROOT/logs"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
DISPLAY="${DISPLAY:-:12.0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-1006}"
export QT_QPA_PLATFORM=xcb

# --- 加载 ROS 环境（宿主的 eco65 工作区在 /home/liangfx/ros2_ws） ---
source /opt/ros/jazzy/setup.bash 2>/dev/null
WS="${ROS2_WS:-/home/liangfx/ros2_ws}"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

stop_stack() {
  echo "[stop] 停止 eco65 display + moveit demo ..."
  pkill -f "[r]eal_moveit_demo" 2>/dev/null
  pkill -f "[e]co65_display" 2>/dev/null
  pkill -f "install/[m]oveit_ros_move_group/move_group" 2>/dev/null
  pkill -f "rm_eco65_config/config/moveit[.]rviz" 2>/dev/null
  sleep 2
  echo "[stop] 完成（宿主 rm_driver/rm_control 未动）。如需回 SNP：docker start snp_automate_2023_real"
  exit 0
}
[ "${1:-}" = "--stop" ] && stop_stack

# --- 前提检查：rm_control 的 FJT action 在不在 ---
echo "[check] 确认真机控制器 FJT action 在线（DOMAIN_ID=$ROS_DOMAIN_ID）..."
if ! timeout 8 ros2 action list 2>/dev/null | grep -q "rm_group_controller/follow_joint_trajectory"; then
  echo "[错误] 没找到 /rm_group_controller/follow_joint_trajectory。"
  echo "       请先启动宿主驱动：./scripts/start_real.sh（或手动起 rm_driver + rm_control）"
  exit 1
fi
echo "[check] FJT action OK"

# --- 停 SNP 容器，独占 /robot_description / TF ---
if docker ps --format '{{.Names}}' | grep -q '^snp_automate_2023_real$'; then
  echo "[info] 停止 SNP 容器（避免双发布）..."
  docker stop snp_automate_2023_real >/dev/null
fi

mkdir -p "$LOG_DIR"
export ROS_DOMAIN_ID DISPLAY

echo "[launch] eco65 RSP (robot_state_publisher) ..."
nohup ros2 launch rm_description rm_eco65_display.launch.py \
  > "$LOG_DIR/eco65_display.log" 2>&1 &
DISP_PID=$!

sleep 3
echo "[launch] eco65 MoveIt demo (move_group + RViz, 窗口在 $DISPLAY) ..."
nohup ros2 launch rm_eco65_config real_moveit_demo.launch.py \
  > "$LOG_DIR/eco65_moveit.log" 2>&1 &
MV_PID=$!

echo
echo "=============================================================="
echo " eco65 原生真机 MoveIt 已启动  (display pid=$DISP_PID, moveit pid=$MV_PID)"
echo " 日志: $LOG_DIR/eco65_display.log  /  eco65_moveit.log"
echo ""
echo " 在 MoveIt RViz(Motion Planning 面板, group=rm_group):"
echo "   1) 确认机械臂模型跟随真机实时位姿"
echo "   2) 拖末端球标设目标 → 点 Plan & Execute → 真机动"
echo " 停止: ./scripts/start_real_moveit.sh --stop"
echo "=============================================================="
