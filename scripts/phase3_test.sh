#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Phase 3.3 真机端到端验证 —— 轨迹生成 → 格式验证 → 真机执行 一键启动
#
# 用法：
#   ./scripts/phase3_test.sh                    # 完整流程
#   ./scripts/phase3_test.sh --verify-only      # 仅验证轨迹格式（不执行真机）
#   ./scripts/phase3_test.sh --generate-only    # 仅生成轨迹
#
# 前提条件：
#   - 真机驱动已启动：./scripts/start_real.sh
#   - 真机ROS_DOMAIN_ID=10
# ============================================================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRAJECTORY_FILE="$ROOT/runtime/snp_home/test_trajectory.yaml"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"

# 颜色输出
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_step() {
  echo -e "${BLUE}>>> $1${NC}"
}

log_success() {
  echo -e "${GREEN}✅ $1${NC}"
}

log_warn() {
  echo -e "${YELLOW}⚠️  $1${NC}"
}

# ============================================================================
# 步骤1：生成轨迹
# ============================================================================
step_generate() {
  log_step "步骤 1/3: 离线生成打磨轨迹"
  python3 "$ROOT/scripts/generate_trajectory.py" "$TRAJECTORY_FILE"
  log_success "轨迹已生成: $TRAJECTORY_FILE"
}

# ============================================================================
# 步骤2：格式验证
# ============================================================================
step_verify() {
  log_step "步骤 2/3: 轨迹格式验证"
  python3 "$ROOT/scripts/convert_trajectory_to_fjt.py" "$TRAJECTORY_FILE"
  log_success "轨迹格式验证通过"
}

# ============================================================================
# 步骤3：真机执行
# ============================================================================
step_execute() {
  log_step "步骤 3/3: 发送轨迹到真机执行"
  log_warn "确认真机已上电、急停在手、网络连通"

  export ROS_DOMAIN_ID
  python3 "$ROOT/scripts/convert_trajectory_to_fjt.py" "$TRAJECTORY_FILE" --execute
  log_success "轨迹执行完成"
}

# ============================================================================
# 主逻辑
# ============================================================================
MODE="${1:-full}"

case "$MODE" in
  "")
    # 完整流程
    log_step "========== Phase 3.3 真机端到端验证 =========="
    log_step "前提: 真机驱动已启动 (./scripts/start_real.sh)"
    echo ""
    step_generate
    echo ""
    step_verify
    echo ""
    step_execute
    echo ""
    log_success "========== 所有步骤完成 =========="
    ;;

  "--verify-only")
    # 仅验证
    log_step "仅验证轨迹格式（跳过生成和执行）"
    if [ ! -f "$TRAJECTORY_FILE" ]; then
      log_warn "轨迹文件不存在，先生成..."
      step_generate
    fi
    step_verify
    ;;

  "--generate-only")
    # 仅生成
    log_step "仅生成轨迹（跳过验证和执行）"
    step_generate
    ;;

  "--execute-only")
    # 仅执行（假设轨迹已存在）
    log_step "仅执行轨迹（假设轨迹已存在）"
    if [ ! -f "$TRAJECTORY_FILE" ]; then
      echo "❌ 轨迹文件不存在: $TRAJECTORY_FILE"
      exit 1
    fi
    step_verify
    step_execute
    ;;

  *)
    echo "用法："
    echo "  $0                  # 完整流程"
    echo "  $0 --verify-only    # 仅验证格式"
    echo "  $0 --generate-only  # 仅生成轨迹"
    echo "  $0 --execute-only   # 仅执行（轨迹必须存在）"
    exit 1
    ;;
esac
