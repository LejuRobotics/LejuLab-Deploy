#!/bin/bash

check_cyclonedds() {
    [ -z "${CYCLONEDDS_URI}" ] && return
    echo "[${NODE_NAME}] CYCLONEDDS_URI=${CYCLONEDDS_URI}"
    if echo "${CYCLONEDDS_URI}" | grep -q "shm"; then
        if [ ! -S /tmp/roudi ]; then
            echo -e "\033[31m[${NODE_NAME}] 错误: iceoryx 共享内存已启用但 RouDi 未运行!\033[0m"
            echo -e "\033[31m[${NODE_NAME}] 请先部署: ./src/leju_launch/scripts/setup_cyclonedds_config.sh\033[0m"
            echo -e "\033[31m[${NODE_NAME}] 或手动启动: ./src/leju_launch/scripts/start_roudi.sh\033[0m"
            exit 1
        fi
    fi
}

TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
LOG_DIR="$HOME/.ros/lejulab/stdout/${TIMESTAMP}"
CORE_DUMP_DIR="$HOME/.ros/lejulab/coredumps/$PPID"
START_ARG_IDX=1

if [ "$1" = "--with-coredump" ]; then
    START_ARG_IDX=2
    NODE_NAME=$(basename "$2")
    # 检查是否有sudo权限, 有则创建coredump目录
    if [ $(id -u) -eq 0 ]; then
        # 设置core dump相关参数
        mkdir -p "${CORE_DUMP_DIR}"
        if [ -d "${CORE_DUMP_DIR}" ]; then
            echo "coredump for kuavo:${NODE_NAME}" >> "${CORE_DUMP_DIR}/README.txt"
        fi
        sudo sysctl -w kernel.core_pattern="${CORE_DUMP_DIR}/core.%e.%p.%t"
        ulimit -c unlimited
    fi
else
    START_ARG_IDX=1
    NODE_NAME=$(basename "$1")
fi

# @stdoutlog
mkdir -p "${LOG_DIR}"
# 所有节点的日志都输出到同一个文件
LOGFILE="${LOG_DIR}/stdout.log" 
# # 每个节点的日志单独输出到文件
# LOGFILE="${LOG_DIR}/${NODE_NAME}.log" 

# 获取当前脚本的父进程ID和名称，输出到stdout.log的开头
echo "PPID: $PPID, NODE_NAME: ${NODE_NAME}" >> "${LOG_DIR}/stdout.log"

check_cyclonedds

# Create a named pipe for non-blocking output logging
FIFO=$(mktemp -u)
mkfifo $FIFO

# Start tee in the background reading from the pipe
tee -a ${LOGFILE} < $FIFO &

# Redirect all output to the pipe and exec the command
exec stdbuf -oL -eL "${@:${START_ARG_IDX}}" > $FIFO 2>&1

# Clean up the pipe (this won't be reached due to exec)
rm -f $FIFO