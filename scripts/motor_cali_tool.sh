#!/bin/bash
#
# 电机校准工具脚本
#
# 用法: ./motor_cali_tool.sh --cali_mode <1|2|3> --cali_type <1|2|3>
# --cali_mode <mode>    : Calibration mode (required):
#                           1: Tooling - 工装校准, 将当前位置记做零点
#                           2: Manual  - 手动校准, 从当前位置 +- 圈数进行校准
#                           3: JointLimited - 基于关节限位校准
#  --cali_type <type>    : Calibration type (required):
#                           1: FullBody  - 全身关节校准
#                           2: UpperBody - 上身校准
#                           3: LowerBody - 下身校准
#                           4: Single    - 单电机校准 (运行中交互选择电机序号)

SCRIPT_DIR=$(dirname "$(realpath "$0")")
# 安装模式（下游闭源仓库）下 LEJULAB_PROJECT_ROOT 由 installed/setup.bash 注入；
# 源码模式（开发者）下回退到 scripts/.. = 仓库根
PROJECT_DIR="${LEJULAB_PROJECT_ROOT:-$(realpath "$SCRIPT_DIR/../")}"
CONFIG_DIR="$PROJECT_DIR/src/leju-hardware"

# 按优先级查找 hw_cali_tool 二进制
TOOL_SEARCH_PATHS=(
    # 安装模式：闭源发布的扁平 bin 树
    "${LEJULAB_INSTALL_ROOT:-$PROJECT_DIR/installed}/bin/hw_cali_tool"
    # 源码模式：开发者本地的 cmake build 目录
    "$PROJECT_DIR/build_cmake/src/leju-hardware/lejusdk-hw/tools/calibration/hw_cali_tool"
    "$PROJECT_DIR/build/src/leju-hardware/lejusdk-hw/tools/calibration/hw_cali_tool"
    # 旧 catkin 路径，留作兼容
    "$PROJECT_DIR/devel/lib/leju-hardware/hw_cali_tool"
)

for tool_path in "${TOOL_SEARCH_PATHS[@]}"; do
    if [ -f "$tool_path" ] && [ -x "$tool_path" ]; then
        exec "$tool_path" --config_dir="$CONFIG_DIR" "$@"
    fi
done

# 最后回退：PATH（source installed/setup.bash 后 hw_cali_tool 已在 PATH 中）
if command -v hw_cali_tool >/dev/null 2>&1; then
    exec hw_cali_tool --config_dir="$CONFIG_DIR" "$@"
fi

echo "错误：未找到 hw_cali_tool"
echo "  - 安装模式：先 'source installed/setup.bash'"
echo "  - 源码模式：先 'cmake --build build_cmake --target hw_cali_tool'"
exit 1