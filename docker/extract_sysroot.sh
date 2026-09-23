#!/usr/bin/env bash
# 从真实 RK3588 板子提取 sysroot 到 docker/sysroot/，供交叉编译镜像烘入。
#
# 用法：
#   ./extract_sysroot.sh                 # 默认板子 test@192.168.28.12，含 drake
#   ./extract_sysroot.sh --without-drake # 跳过 drake
#   BOARD_HOST=192.168.50.239 ./extract_sysroot.sh
#
# 依赖宿主：sshpass、rsync、python3。板子需 passwordless sudo（用于读全 /usr/lib）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BOARD_USER="${BOARD_USER:-test}"
BOARD_HOST="${BOARD_HOST:-192.168.28.12}"
if [ -z "${BOARD_PASS:-}" ]; then
    echo "错误: 请设置 BOARD_PASS 环境变量" >&2
    exit 1
fi
BOARD_DRAKE="${BOARD_DRAKE:-/home/test/drake_install_prefix}"
SYSROOT="${SYSROOT:-$SCRIPT_DIR/sysroot}"
WITH_DRAKE=1

for arg in "$@"; do
    case "$arg" in
        --without-drake) WITH_DRAKE=0 ;;
        *) echo "未知参数: $arg" >&2; exit 1 ;;
    esac
done

for tool in sshpass rsync python3; do
    command -v "$tool" >/dev/null || { echo "缺少宿主依赖: $tool" >&2; exit 1; }
done

SSH_CMD="sshpass -p $BOARD_PASS ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10"
RSYNC_OPTS=(-a --info=progress2 --delete --rsync-path="sudo rsync" -e "$SSH_CMD")

# 排除非链接所需的大目录，缩小 sysroot
EXCLUDES_FILE="$(mktemp)"
trap 'rm -f "$EXCLUDES_FILE"' EXIT
cat >"$EXCLUDES_FILE" <<'EOF'
modules/
firmware/
locale/
locales/
python2.7/
__pycache__/
*.pyc
*.pyo
doc/
man/
info/
icons/
fonts/
themes/
help/
sounds/
backgrounds/
lintian/
bug/
gtk-doc/
EOF

echo "=== 提取 sysroot 自 ${BOARD_USER}@${BOARD_HOST} -> ${SYSROOT} ==="
mkdir -p "$SYSROOT/usr/include" "$SYSROOT/usr/lib" "$SYSROOT/usr/local" "$SYSROOT/usr/share" "$SYSROOT/opt"

pull() { # 板子源目录(带/) 本地目标(带/)
    echo "--- rsync $1"
    rsync "${RSYNC_OPTS[@]}" --exclude-from="$EXCLUDES_FILE" \
        "${BOARD_USER}@${BOARD_HOST}:$1" "$2"
}

pull /usr/include/   "$SYSROOT/usr/include/"
pull /usr/lib/       "$SYSROOT/usr/lib/"
pull /usr/local/     "$SYSROOT/usr/local/"
pull /usr/share/     "$SYSROOT/usr/share/"   # cmake/pkgconfig config (eigen3 等), 已排除 doc/man/icons

if [ "$WITH_DRAKE" -eq 1 ]; then
    echo "=== 提取 drake -> /opt/drake ==="
    rsync "${RSYNC_OPTS[@]}" "${BOARD_USER}@${BOARD_HOST}:${BOARD_DRAKE}/" "$SYSROOT/opt/drake/"
else
    echo "=== 跳过 drake (--without-drake) ==="
fi

# 复刻板子 usrmerge：/lib -> usr/lib，使指向 /lib/... 的绝对软链能在 sysroot 内解析
ln -sfn usr/lib "$SYSROOT/lib"
ln -sfn usr/bin "$SYSROOT/bin"

echo "=== 相对化绝对软链（关键：否则交叉链接器在 sysroot 外找目标） ==="
python3 - "$SYSROOT" <<'PY'
import os, sys
sysroot = os.path.realpath(sys.argv[1])
fixed = 0
for root, dirs, files in os.walk(sysroot):
    for name in dirs + files:
        p = os.path.join(root, name)
        if not os.path.islink(p):
            continue
        tgt = os.readlink(p)
        if not tgt.startswith('/'):
            continue  # 已是相对，跳过
        # 绝对目标视为相对 sysroot 根
        in_sysroot = os.path.join(sysroot, tgt.lstrip('/'))
        rel = os.path.relpath(in_sysroot, os.path.dirname(p))
        os.remove(p)
        os.symlink(rel, p)
        fixed += 1
print(f"  相对化软链: {fixed} 个")
PY

echo "=== 完成 ==="
du -sh "$SYSROOT" 2>/dev/null || true
echo "sysroot: $SYSROOT"
