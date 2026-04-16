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

SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../")

# 保存原始参数并清空当前脚本的参数
ORIGINAL_ARGS=("$@")
set --

# 定义工具路径和对应的setup路径
declare -A TOOL_ENV_MAP=(
    ["$PROJECT_DIR/devel/lib/leju-hardware/hw_cali_tool"]="$PROJECT_DIR/devel/setup.bash"
    ["$PROJECT_DIR/installed/bin/hw_cali_tool"]="$PROJECT_DIR/installed/setup.bash"
)

# 查找并执行工具
for tool_path in "${!TOOL_ENV_MAP[@]}"; do
    if [ -f "$tool_path" ] && [ -x "$tool_path" ]; then
        # Source对应的环境
        [ -f "${TOOL_ENV_MAP[$tool_path]}" ] && source "${TOOL_ENV_MAP[$tool_path]}"

        # 恢复参数
        set -- "${ORIGINAL_ARGS[@]}"

        # 执行工具
        exec "$tool_path" --config_dir="$(rospack find leju-hardware)" "$@"
    fi
done


echo "错误：未找到 hw_cali_tool, 请先 catkin build 构建项目"
exit 1