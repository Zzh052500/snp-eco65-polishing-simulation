#!/usr/bin/env bash
set -euo pipefail
# 真机（RM-ECO65）启动脚本 —— 与仿真完全隔离（DOMAIN_ID=10，仿真用 7）
# 只起 SNP 需要的：rm_driver(关节状态) + rm_control(rm_group_controller FJT action)
# 不起 rm_bringup 的 real_moveit_demo（多余的 move_group + rviz），避免与 SNP Tesseract 规划器冲突
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export CURRENT_UID="$(id -u):$(id -g)"
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
# 真机域：宿主驱动与真机容器共用 10，避免和仿真(7)互相污染
export ROS_DOMAIN_ID=10

echo ">>> [真机] source ros2_ws（DOMAIN_ID=${ROS_DOMAIN_ID}）..."
set +u    # colcon 的 setup.bash 依赖未定义变量（如 COLCON_TRACE），先关掉严格检查
source /home/liangfx/ros2_ws/install/setup.bash
set -u

echo ">>> [真机] 启动 rm_driver（真机关节状态 /joint_states，TCP连控制器 + UDP收上报）..."
setsid nohup ros2 launch rm_driver rm_eco65_driver.launch.py > /tmp/rm_driver.log 2>&1 &
DRIVER_PID=$!
echo "    rm_driver PID=${DRIVER_PID} (log: /tmp/rm_driver.log)"

echo ">>> [真机] 启动 rm_control（rm_group_controller/follow_joint_trajectory）..."
setsid nohup ros2 launch rm_control rm_eco65_control.launch.py > /tmp/rm_control.log 2>&1 &
CONTROL_PID=$!
echo "    rm_control PID=${CONTROL_PID} (log: /tmp/rm_control.log)"

echo ">>> [真机] 启动真机容器（compose.real.yml，DOMAIN_ID=${ROS_DOMAIN_ID}）..."
docker rm -f snp_automate_2023_real >/dev/null 2>&1 || true
docker compose -f "$ROOT/docker/compose.real.yml" up -d

echo ""
echo ">>> [真机] 验证（约 30s 后）..."
sleep 30

echo "--- ① /joint_states 是否流过来（验证 udp_ip 修复，最重要） ---"
docker exec snp_automate_2023_real bash -lc \
  'source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=10 timeout 5 ros2 topic echo /joint_states --once 2>/dev/null' \
  | grep -E "name|position|data:" | head -15 || echo "未收到 /joint_states —— 检查 udp_ip / 控制器上报"

echo ""
echo "--- ② rm_group_controller action 是否注册 ---"
docker exec snp_automate_2023_real bash -lc \
  'source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=10 timeout 5 ros2 action list 2>/dev/null' \
  | grep -E "rm_group_controller" && echo "FJT action OK" || echo "未发现 rm_group_controller action —— 检查 rm_control"

echo ""
echo "--- ③ 容器内 SNP 节点是否起来 ---"
docker exec snp_automate_2023_real bash -lc \
  'source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=10 timeout 5 ros2 node list 2>/dev/null' \
  | grep -E "runtime_monitor|rviz|fjt_proxy|planning|noether|snp" || echo "（未匹配到，可能节点名不同）"

echo ""
echo ">>> [真机] 说明："
echo "    rm_driver PID=${DRIVER_PID}，rm_control PID=${CONTROL_PID} 在后台运行。"
echo "    停止：kill ${DRIVER_PID} ${CONTROL_PID} && docker stop snp_automate_2023_real"
