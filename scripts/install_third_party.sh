#!/usr/bin/env bash

set -euo pipefail

DRAKE_VERSION="1.19.0-1"
PINOCCHIO_VERSION="2.6.21-1focal.20240830.092123"
ROS_SNAPSHOT_DATE="2024-10-31"
ROS_SNAPSHOT_KEY="4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
  echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
  echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
  echo -e "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $*" >&2
}

target_user() {
  if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
    echo "${SUDO_USER}"
  else
    id -un
  fi
}

target_home() {
  getent passwd "$(target_user)" | cut -d: -f6
}

ensure_shell_block() {
  local file="$1"
  local begin_marker="$2"
  local end_marker="$3"
  local block_file="$4"

  mkdir -p "$(dirname "${file}")"
  touch "${file}"

  python3 - "$file" "$begin_marker" "$end_marker" "$block_file" <<'PY'
import pathlib
import sys

target = pathlib.Path(sys.argv[1])
begin = sys.argv[2]
end = sys.argv[3]
block_path = pathlib.Path(sys.argv[4])
block = block_path.read_text()
content = target.read_text()

start = content.find(begin)
finish = content.find(end)

replacement = f"{begin}\n{block}{end}\n"

if start != -1 and finish != -1 and finish >= start:
    finish += len(end)
    if finish < len(content) and content[finish] == "\n":
        finish += 1
    updated = content[:start] + replacement + content[finish:]
else:
    if content and not content.endswith("\n"):
        content += "\n"
    updated = content + ("\n" if content else "") + replacement

target.write_text(updated)
PY
}

check_system() {
  if [[ ! -f /etc/os-release ]]; then
    log_error "无法检测当前系统。"
    exit 1
  fi

  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID}" != "ubuntu" ]]; then
    log_error "该脚本仅支持 Ubuntu，当前系统为 ${ID}。"
    exit 1
  fi

  if [[ "${VERSION_ID}" != "20.04" ]]; then
    log_warning "当前系统为 Ubuntu ${VERSION_ID}，脚本按 Ubuntu 20.04 设计。"
  fi
}

apt_update() {
  log_info "更新 apt 包索引..."
  sudo apt-get update -y
}

install_apt_packages() {
  local packages=("$@")
  if [[ ${#packages[@]} -eq 0 ]]; then
    return 0
  fi

  log_info "安装 apt 依赖: ${packages[*]}"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${packages[@]}"
}

install_base_packages() {
  local packages=(
    build-essential
    ca-certificates
    cmake
    curl
    gnupg
    libacl1-dev
    libboost-all-dev
    libeigen3-dev
    libgflags-dev
    libgl1-mesa-dev
    libgl1-mesa-glx
    libglew-dev
    libglfw3-dev
    libgoogle-glog-dev
    liblcm-dev
    liblmdb-dev
    libmodbus-dev
    libncurses5-dev
    libosmesa6-dev
    libprotobuf-c-dev
    libprotobuf-dev
    libtinyxml2-dev
    libusb-1.0-0-dev
    libyaml-cpp-dev
    lsb-release
    pkg-config
    protobuf-compiler
    wget
    xserver-xorg-dev
  )

  install_apt_packages "${packages[@]}"
}

install_openvino() {
  log_info "配置 OpenVINO apt 源..."

  local key_file
  key_file="$(mktemp)"
  wget -qO "${key_file}" \
    https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
  sudo gpg --yes --dearmor --output /etc/apt/trusted.gpg.d/intel.gpg "${key_file}"
  rm -f "${key_file}"

  echo "deb https://apt.repos.intel.com/openvino ubuntu20 main" \
    | sudo tee /etc/apt/sources.list.d/intel-openvino.list >/dev/null

  apt_update
  install_apt_packages openvino
}

install_drake() {
  log_info "配置 Drake apt 源..."

  wget -qO- https://drake-apt.csail.mit.edu/drake.asc \
    | gpg --dearmor \
    | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null

  echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/$(lsb_release -cs) $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null

  apt_update

  if ! apt-cache madison drake-dev | awk '{print $3}' | grep -Fxq "${DRAKE_VERSION}"; then
    log_error "当前 Drake apt 源中未找到 drake-dev=${DRAKE_VERSION}。"
    log_error "为保证版本一致性，脚本不会回退安装其他版本。"
    exit 1
  fi

  install_apt_packages "drake-dev=${DRAKE_VERSION}"

  local installed_version
  installed_version="$(dpkg-query -W -f='${Version}' drake-dev 2>/dev/null || true)"
  if [[ "${installed_version}" != "${DRAKE_VERSION}" ]]; then
    log_error "Drake 安装结果校验失败，期望 ${DRAKE_VERSION}，实际为 ${installed_version:-<未安装>}。"
    exit 1
  fi

  log_success "Drake 已锁定安装为 ${installed_version}"
}

install_pinocchio() {
  log_info "配置 ROS snapshot apt 源 (${ROS_SNAPSHOT_DATE}) 以锁定 Pinocchio 版本..."

  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key "${ROS_SNAPSHOT_KEY}"

  echo "deb http://snapshots.ros.org/noetic/${ROS_SNAPSHOT_DATE}/ubuntu $(lsb_release -sc) main" \
    | sudo tee /etc/apt/sources.list.d/ros-snapshots.list >/dev/null

  apt_update

  if ! apt-cache madison ros-noetic-pinocchio | awk '{print $3}' | grep -Fxq "${PINOCCHIO_VERSION}"; then
    log_error "当前 ROS snapshot 源中未找到 ros-noetic-pinocchio=${PINOCCHIO_VERSION}。"
    log_error "为保证版本一致性，脚本不会回退安装其他版本。"
    exit 1
  fi

  log_info "安装 ros-noetic-pinocchio=${PINOCCHIO_VERSION} (--reinstall)"
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --reinstall \
    "ros-noetic-pinocchio=${PINOCCHIO_VERSION}"

  local installed_version
  installed_version="$(dpkg-query -W -f='${Version}' ros-noetic-pinocchio 2>/dev/null || true)"
  if [[ "${installed_version}" != "${PINOCCHIO_VERSION}" ]]; then
    log_error "Pinocchio 安装结果校验失败，期望 ${PINOCCHIO_VERSION}，实际为 ${installed_version:-<未安装>}。"
    exit 1
  fi

  log_success "Pinocchio 已锁定安装为 ${installed_version}"
}

configure_shell_env() {
  local user_home
  user_home="$(target_home)"
  local bashrc="${user_home}/.bashrc"
  local zshrc="${user_home}/.zshrc"
  local block_file
  block_file="$(mktemp)"

  cat > "${block_file}" <<'EOF'
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{}.{}".format(sys.version_info[0], sys.version_info[1]))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="/opt/drake/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
if [ -f /opt/intel/openvino/setupvars.sh ]; then
  # shellcheck disable=SC1091
  source /opt/intel/openvino/setupvars.sh
elif [ -d /opt/intel/openvino/runtime/cmake ]; then
  export OpenVINO_DIR="/opt/intel/openvino/runtime/cmake"
fi
EOF

  ensure_shell_block \
    "${bashrc}" \
    "# >>> lejulab third-party deps >>>" \
    "# <<< lejulab third-party deps <<<" \
    "${block_file}"

  ensure_shell_block \
    "${zshrc}" \
    "# >>> lejulab third-party deps >>>" \
    "# <<< lejulab third-party deps <<<" \
    "${block_file}"

  rm -f "${block_file}"

  log_success "已写入 Drake / OpenVINO 环境变量到 ${bashrc} 和 ${zshrc}"
}

print_summary() {
  cat <<'EOF'

安装完成。已覆盖的系统侧第三方依赖包括：
  - Drake
  - OpenVINO
  - Pinocchio
  - Eigen3 / yaml-cpp / Protobuf / protobuf-c
  - GLFW / Mesa / OSMesa / GLEW
  - LCM / gflags / glog / LMDB
  - ACL / libusb / modbus / Boost / tinyxml2 / ncurses

下一步建议：
  1. 重新打开终端，或执行: source ~/.bashrc
  2. 回到工作区后执行: catkin build
  3. 如果要启用 iceoryx 共享内存，再执行:
     ./src/leju_launch/scripts/setup_cyclonedds_config.sh
EOF
}

main() {
  check_system
  apt_update
  install_base_packages
  install_openvino
  install_drake
  install_pinocchio
  configure_shell_env
  print_summary
}

main "$@"
