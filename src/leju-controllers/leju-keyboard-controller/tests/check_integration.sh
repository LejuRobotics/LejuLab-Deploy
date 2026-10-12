#!/usr/bin/env bash
# check_integration.sh — 键盘控制器集成检测
#
# 用法:
#   bash src/leju-controllers/leju-keyboard-controller/tests/check_integration.sh [build_dir]
#   bash tests/check_integration.sh                    # 从组件目录运行
#   bash tests/check_integration.sh /path/to/build     # 指定构建目录
#
# 不需要硬件，纯离线检测。
#
# 检查项 (7 大类, 共 ~32 项):
#   [1] 源文件完整性 — motor_keyboard_ctrl.cpp, keyboard_ctrl_utils.h, CMakeLists.txt 存在
#   [2] 编译检查     — 二进制 motor_keyboard_ctrl / test_keyboard_ctrl 是否可执行
#   [3] 命令行参数   — --help 输出包含 --step / --kp / --csv 等关键参数说明
#   [4] launch 脚本  — 语法正确、支持 -- 分隔符、setsid 隔离、SIGINT 隔离、cleanup 轮询
#   [5] 功能完整性   — 源码扫描: SineTest、CsvLogger、SCHED_FIFO、回零插值、按键处理
#   [6] 数学公式     — 正弦公式 sin(2π·f·t)、deg→rad 转换、center_rad 中心偏移
#   [7] 单元测试     — 运行 GTest (需 BUILD_TESTING=ON 编译, 否则 SKIP)
#
# 判定:
#   PASS — 检查通过
#   FAIL — 检查失败 (脚本最终 exit 1)
#   SKIP — 前置条件不满足 (如未编译测试二进制), 不算失败

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
BUILD_DIR="${1:-${PROJ_ROOT}/build_cmake}"

PASS=0
FAIL=0
SKIP=0

pass() { echo "  [PASS] $1"; PASS=$((PASS+1)); }
fail() { echo "  [FAIL] $1: $2"; FAIL=$((FAIL+1)); }
skip() { echo "  [SKIP] $1: $2"; SKIP=$((SKIP+1)); }

echo "============================================"
echo " 键盘控制器集成检测"
echo " 项目: ${PROJ_ROOT}"
echo " 构建: ${BUILD_DIR}"
echo "============================================"
echo ""

# ---- 1. 源文件完整性 ----
echo "[1] 源文件完整性检查"

SRC="${PROJ_ROOT}/src/leju-controllers/leju-keyboard-controller"
if [[ -f "${SRC}/motor_keyboard_ctrl.cpp" ]]; then
    pass "motor_keyboard_ctrl.cpp 存在"
else
    fail "motor_keyboard_ctrl.cpp" "文件不存在"
fi

if [[ -f "${SRC}/keyboard_ctrl_utils.h" ]]; then
    pass "keyboard_ctrl_utils.h 存在"
else
    fail "keyboard_ctrl_utils.h" "文件不存在"
fi

if [[ -f "${SRC}/CMakeLists.txt" ]]; then
    pass "CMakeLists.txt 存在"
else
    fail "CMakeLists.txt" "文件不存在"
fi

# 检查头文件被正确 include
if grep -q '#include "keyboard_ctrl_utils.h"' "${SRC}/motor_keyboard_ctrl.cpp"; then
    pass "motor_keyboard_ctrl.cpp 引用 keyboard_ctrl_utils.h"
else
    fail "include" "motor_keyboard_ctrl.cpp 未引用 keyboard_ctrl_utils.h"
fi

echo ""

# ---- 2. 编译检查 ----
echo "[2] 编译检查"

BIN="${BUILD_DIR}/src/leju-controllers/leju-keyboard-controller/motor_keyboard_ctrl"
if [[ -x "${BIN}" ]]; then
    pass "motor_keyboard_ctrl 二进制存在且可执行"
else
    fail "binary" "motor_keyboard_ctrl 未找到或不可执行"
fi

TEST_BIN="${BUILD_DIR}/src/leju-controllers/leju-keyboard-controller/test_keyboard_ctrl"
if [[ -x "${TEST_BIN}" ]]; then
    pass "test_keyboard_ctrl 测试二进制存在"
else
    skip "test_keyboard_ctrl" "未编译 (需要 -DBUILD_TESTING=ON)"
fi

echo ""

# ---- 3. --help 参数测试 ----
echo "[3] 命令行参数检查"

if [[ -x "${BIN}" ]]; then
    HELP_OUT=$("${BIN}" --help 2>&1 || true)

    if echo "${HELP_OUT}" | grep -q -- "--step"; then
        pass "--help 包含 --step 参数说明"
    else
        fail "--help" "缺少 --step 参数说明"
    fi

    if echo "${HELP_OUT}" | grep -q -- "--kp"; then
        pass "--help 包含 --kp 参数说明"
    else
        fail "--help" "缺少 --kp 参数说明"
    fi

    if echo "${HELP_OUT}" | grep -q -- "--csv"; then
        pass "--help 包含 --csv 参数说明"
    else
        fail "--help" "缺少 --csv 参数说明"
    fi

    if echo "${HELP_OUT}" | grep -q -- "--hold-mode"; then
        pass "--help 包含 --hold-mode 参数说明"
    else
        fail "--help" "缺少 --hold-mode 参数说明"
    fi
else
    skip "命令行参数" "二进制不存在"
fi

echo ""

# ---- 4. launch 脚本检查 ----
echo "[4] launch 脚本检查"

LAUNCH="${PROJ_ROOT}/src/leju_launch/scripts/launch_keyboard_ctrl.sh"
if [[ -f "${LAUNCH}" ]]; then
    pass "launch_keyboard_ctrl.sh 存在"

    # 语法检查
    if bash -n "${LAUNCH}" 2>/dev/null; then
        pass "launch 脚本语法正确"
    else
        fail "launch 语法" "bash -n 检查失败"
    fi

    # 检查 -- 分隔符支持
    if grep -q 'found_separator' "${LAUNCH}"; then
        pass "launch 支持 -- 参数分隔符"
    else
        fail "launch" "缺少 -- 参数分隔符支持"
    fi

    # 检查进程组隔离 (setsid)
    if grep -q 'setsid' "${LAUNCH}"; then
        pass "launch 使用 setsid 隔离 hardware_node"
    else
        fail "launch" "缺少 setsid 隔离"
    fi

    # 检查清理函数等待时间 (不是硬编码的 sleep 2)
    if grep -q 'sleep 0.5' "${LAUNCH}"; then
        pass "launch cleanup 使用轮询等待 (非固定 sleep 2)"
    else
        fail "launch" "cleanup 可能使用固定 sleep 导致电机失能被中断"
    fi

    # 检查 SIGINT 隔离
    if grep -q "trap '' INT" "${LAUNCH}"; then
        pass "launch 脚本隔离 SIGINT"
    else
        fail "launch" "缺少 SIGINT 隔离"
    fi
else
    fail "launch" "launch_keyboard_ctrl.sh 不存在"
fi

echo ""

# ---- 5. 功能完整性检查 (源码关键字) ----
echo "[5] 功能完整性检查 (源码扫描)"

SRC_FILE="${SRC}/motor_keyboard_ctrl.cpp"
UTILS_FILE="${SRC}/keyboard_ctrl_utils.h"

# 正弦测试功能
if grep -q 'SineTest' "${SRC_FILE}"; then
    pass "正弦测试功能存在"
else
    fail "正弦测试" "未找到 SineTest"
fi

if grep -q 'sine_tests' "${SRC_FILE}"; then
    pass "正弦测试状态数组"
else
    fail "正弦测试" "未找到 sine_tests 数组"
fi

# CSV 记录
if grep -q 'CsvLogger' "${SRC_FILE}"; then
    pass "CSV 记录功能存在"
else
    fail "CSV 记录" "未找到 CsvLogger"
fi

# SCHED_FIFO 实时调度
if grep -q 'SCHED_FIFO' "${SRC_FILE}"; then
    pass "SCHED_FIFO 实时调度"
else
    fail "实时调度" "未找到 SCHED_FIFO"
fi

# 电机失能等待
if grep -q 'HardwareState::STOPPED' "${SRC_FILE}"; then
    pass "退出时等待电机失能完成"
else
    fail "电机失能" "退出时未等待 STOPPED 状态"
fi

# 回零插值
if grep -q 'interpolateBack' "${SRC_FILE}"; then
    pass "退出前回零插值"
else
    fail "回零" "未找到 interpolateBack"
fi

# publishStopRobot
if grep -q 'publishStopRobot' "${SRC_FILE}"; then
    pass "退出时发送停止命令"
else
    fail "停止命令" "未找到 publishStopRobot"
fi

# 键盘按键: W/S, T, E/R, F/G, Q, Space
for key_check in "'w'" "'t'" "'e'" "'f'" "'q'" "' '"; do
    if grep -q "${key_check}" "${SRC_FILE}"; then
        pass "按键处理: ${key_check}"
    else
        fail "按键" "未找到 ${key_check} 处理"
    fi
done

echo ""

# ---- 6. SineTest 数学验证 (头文件) ----
echo "[6] SineTest 数学公式检查"

if grep -q 'std::sin(2.0 \* M_PI \* frequency_hz \* t)' "${UTILS_FILE}"; then
    pass "正弦公式: sin(2π·f·t)"
else
    fail "正弦公式" "未找到正确的正弦公式"
fi

if grep -q 'amplitude_rad \* std::sin' "${UTILS_FILE}"; then
    pass "振幅使用弧度: amplitude_rad * sin(...)"
else
    fail "振幅单位" "未找到 amplitude_rad * sin 模式"
fi

if grep -q 'center_rad' "${UTILS_FILE}"; then
    pass "正弦以 center_rad 为中心"
else
    fail "正弦中心" "未找到 center_rad"
fi

echo ""

# ---- 7. 单元测试运行 ----
echo "[7] 单元测试"

if [[ -x "${TEST_BIN}" ]]; then
    if TEST_OUT=$("${TEST_BIN}" --gtest_color=no 2>&1); then
        PASSED=$(echo "${TEST_OUT}" | grep -oP '\d+ tests?' | tail -1)
        pass "GTest 全部通过 (${PASSED})"
    else
        fail "GTest" "部分测试失败"
        echo "${TEST_OUT}" | grep -E '\[  FAILED  \]' || true
    fi
else
    skip "单元测试" "test_keyboard_ctrl 未编译"
fi

echo ""

# ---- 汇总 ----
echo "============================================"
echo " 结果: ${PASS} PASS / ${FAIL} FAIL / ${SKIP} SKIP"
echo "============================================"

if [[ ${FAIL} -gt 0 ]]; then
    exit 1
fi
exit 0
