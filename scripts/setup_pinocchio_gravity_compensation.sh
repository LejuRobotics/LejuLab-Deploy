#!/usr/bin/env bash
# 面向 GitCode 开源树 Lejulab_ARM（按仓库实际目录）：
#   控制器只有 installed/bin/run_rl_controller，src/leju-controllers/leju-rl-controller 只有 config
#   SDK 头文件 installed/include，库 installed/lib/liblejusdk-vr.so
#   CycloneDDS / iceoryx 在 src/lejusdk/3rd_party/<aarch64|x86_64>
# 开源树的 CMake 不认识 ENABLE_PINOCCHIO，也没有 run_rl_controller 目标，本脚本不编译控制器。
# 闭源开发树如果还带着控制器源码，脚本会改走源码编译。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${PROJECT_DIR}/build}"
URDF_PATH="${PROJECT_DIR}/src/leju_assets/models/biped_s17/urdf/biped_s17.urdf"
INSTALL_ROOT="${PROJECT_DIR}/installed"

usage() {
  cat <<EOF
Usage: bash scripts/setup_pinocchio_gravity_compensation.sh

在开源仓库 Lejulab_ARM 根目录执行。安装 Pinocchio，并用 installed/ 里的
SDK 编译 scripts/arm_traj_tracking_test.cpp。不启动机器人，也不编译控制器。

用到的路径：
  scripts/setup_pinocchio_gravity_compensation.sh
  scripts/arm_traj_tracking_test.cpp
  scripts/arm_traj_tracking.yaml
  installed/bin/run_rl_controller
  installed/include  installed/lib
  src/lejusdk/3rd_party/aarch64   （在 x86 机器上则是 x86_64）
  src/leju_assets/models/biped_s17/urdf/biped_s17.urdf
  src/leju_launch/scripts/launch_real.sh
EOF
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "ERROR: root privileges are required to install packages." >&2
    exit 1
  fi
}

pick_ros_distro() {
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "${ROS_DISTRO}"
    return
  fi
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo humble
    return
  fi
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    echo noetic
    return
  fi
  return 1
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (($#)); then
  echo "ERROR: unexpected argument: $1" >&2
  usage >&2
  exit 1
fi

TEST_CPP=""
TEST_YAML=""
for pair in \
  "${SCRIPT_DIR}/arm_traj_tracking_test.cpp|${SCRIPT_DIR}/arm_traj_tracking.yaml" \
  "${PROJECT_DIR}/src/lejusdk/examples/arm_traj_tracking_test.cpp|${PROJECT_DIR}/src/lejusdk/examples/arm_traj_tracking.yaml" \
  "${PROJECT_DIR}/src/lejusdk/lejusdk-vr/examples/arm_traj_tracking_test.cpp|${PROJECT_DIR}/src/lejusdk/lejusdk-vr/examples/arm_traj_tracking.yaml"
do
  cpp="${pair%%|*}"
  yaml="${pair##*|}"
  if [[ -f "${cpp}" && -f "${yaml}" ]]; then
    TEST_CPP="${cpp}"
    TEST_YAML="${yaml}"
    break
  fi
done
if [[ -z "${TEST_CPP}" ]]; then
  echo "ERROR: arm_traj_tracking_test.cpp and arm_traj_tracking.yaml were not found." >&2
  echo "Put them in scripts/ (opensource tree does not contain src/lejusdk/lejusdk-vr)." >&2
  exit 1
fi
if [[ ! -f "${URDF_PATH}" ]]; then
  echo "ERROR: v17 URDF not found: ${URDF_PATH}" >&2
  exit 1
fi

OPENSOURCE=0
CTRL_SRC_CMAKE="${PROJECT_DIR}/src/leju-controllers/leju-rl-controller/CMakeLists.txt"
if [[ -x "${INSTALL_ROOT}/bin/run_rl_controller" && ! -f "${CTRL_SRC_CMAKE}" ]]; then
  OPENSOURCE=1
fi

ROS_DISTRO_SELECTED="$(pick_ros_distro)" || {
  echo "ERROR: neither /opt/ros/humble nor /opt/ros/noetic is installed." >&2
  exit 1
}
ROS_SETUP="/opt/ros/${ROS_DISTRO_SELECTED}/setup.bash"
PINOCCHIO_PKG="ros-${ROS_DISTRO_SELECTED}-pinocchio"
ROS_LIB="/opt/ros/${ROS_DISTRO_SELECTED}/lib"

if ! dpkg-query -W -f='${Status}' "${PINOCCHIO_PKG}" 2>/dev/null | grep -q 'install ok installed'; then
  echo "Installing ${PINOCCHIO_PKG}..."
  run_as_root apt-get update
  run_as_root apt-get install -y "${PINOCCHIO_PKG}"
fi
if ! dpkg-query -W -f='${Status}' libyaml-cpp-dev 2>/dev/null | grep -q 'install ok installed'; then
  echo "Installing libyaml-cpp-dev..."
  run_as_root apt-get update
  run_as_root apt-get install -y libyaml-cpp-dev
fi

set +u
source "${ROS_SETUP}"
set -u
export CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO_SELECTED}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
if [[ -f "${INSTALL_ROOT}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${INSTALL_ROOT}/setup.bash"
  set -u
fi
export LD_LIBRARY_PATH="${ROS_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

host_arch() {
  case "$(uname -m)" in
    aarch64|arm64) echo aarch64 ;;
    x86_64) echo x86_64 ;;
    *)
      echo "ERROR: unsupported architecture: $(uname -m)" >&2
      return 1
      ;;
  esac
}

elf_arch() {
  local desc
  desc="$(file -b "$1")"
  if [[ "${desc}" == *"ARM aarch64"* ]]; then
    echo aarch64
  elif [[ "${desc}" == *"x86-64"* ]]; then
    echo x86_64
  else
    echo unknown
  fi
}

CTRL_BIN=""
PINOCCHIO_OK=0
pinocchio_link=""
if [[ "${OPENSOURCE}" -eq 1 ]]; then
  CTRL_BIN="${INSTALL_ROOT}/bin/run_rl_controller"
  echo "开源树：使用预编译 ${CTRL_BIN}"
  echo "本仓库没有 leju-rl-controller 源码，不执行 cmake -DENABLE_PINOCCHIO，也不编译控制器。"
else
  echo "闭源树：用 Pinocchio 配置并编译 run_rl_controller (${ROS_DISTRO_SELECTED})..."
  CONFIG_LOG="$(mktemp)"
  trap 'rm -f "${CONFIG_LOG}"' EXIT
  cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -DENABLE_PINOCCHIO=ON 2>&1 | tee "${CONFIG_LOG}"
  if ! grep -Fq 'Pinocchio found: gravity compensation ENABLED' "${CONFIG_LOG}"; then
    echo "ERROR: CMake did not enable Pinocchio gravity compensation; build aborted." >&2
    exit 1
  fi
  rm -f "${CONFIG_LOG}"
  trap - EXIT
  echo "Building run_rl_controller..."
  cmake --build "${BUILD_DIR}" --target run_rl_controller --parallel "$(nproc)"
  CTRL_BIN="${BUILD_DIR}/src/leju-controllers/leju-rl-controller/run_rl_controller"
fi

if [[ ! -x "${CTRL_BIN}" ]]; then
  echo "ERROR: controller binary not found: ${CTRL_BIN}" >&2
  exit 1
fi

if readelf -d "${CTRL_BIN}" 2>/dev/null | grep -q 'libpinocchio'; then
  PINOCCHIO_OK=1
  pinocchio_link="$(readelf -d "${CTRL_BIN}" | grep 'libpinocchio' || true)"
  if [[ "$(elf_arch "${CTRL_BIN}")" == "$(host_arch)" ]]; then
    ldd_line="$(ldd "${CTRL_BIN}" | grep 'libpinocchio' || true)"
    if [[ -n "${ldd_line}" ]]; then
      pinocchio_link="${ldd_line}"
    fi
    if grep -q 'not found' <<<"${pinocchio_link}"; then
      echo "ERROR: 控制器链了 libpinocchio，但当前找不到这个库：" >&2
      echo "  ${pinocchio_link}" >&2
      echo "export LD_LIBRARY_PATH=${ROS_LIB}:\${LD_LIBRARY_PATH}" >&2
      exit 1
    fi
  fi
elif [[ "${OPENSOURCE}" -eq 0 ]]; then
  echo "ERROR: ${CTRL_BIN} is not linked to libpinocchio." >&2
  exit 1
else
  echo "当前 installed/bin/run_rl_controller 没有链 libpinocchio，前馈力矩会是 0。"
  echo "开源仓库不能重编这个文件。需要从公司仓库 lejulab_platform 的 aarch64 编译结果替换它。"
  echo "本脚本继续编译测试程序。"
fi

build_test_opensource() {
  local arch tp cxx out vr_lib iox_inc sdk_arch
  arch="$(host_arch)"
  tp="${PROJECT_DIR}/src/lejusdk/3rd_party/${arch}"
  vr_lib="${INSTALL_ROOT}/lib/liblejusdk-vr.so"
  sdk_arch="$(elf_arch "${vr_lib}")"
  if [[ "${sdk_arch}" != "${arch}" ]]; then
    echo "ERROR: installed/lib 是 ${sdk_arch}，当前机器是 ${arch}，不能在这里链接测试程序。" >&2
    echo "请在与 installed/bin/run_rl_controller 相同架构的机器上运行本脚本。" >&2
    return 1
  fi
  if [[ ! -d "${tp}/cyclonedds-0.10.2" || ! -d "${tp}/cyclonedds-cxx-0.10.2" ]]; then
    echo "ERROR: 缺少 ${tp}/cyclonedds-0.10.2 或 cyclonedds-cxx-0.10.2" >&2
    return 1
  fi
  if [[ -d "${tp}/iceoryx/include/iceoryx/v" ]]; then
    iox_inc="${tp}/iceoryx/include/iceoryx/v"
  else
    iox_inc="$(find "${tp}/iceoryx/include/iceoryx" -mindepth 1 -maxdepth 1 -type d -name 'v*' | head -1)"
  fi
  if [[ -z "${iox_inc}" || ! -d "${iox_inc}" ]]; then
    echo "ERROR: 找不到 iceoryx 头文件目录: ${tp}/iceoryx/include" >&2
    return 1
  fi
  cxx="$(command -v g++-11 || command -v g++)"
  out="${BUILD_DIR}/arm_traj_tracking_test"
  mkdir -p "${BUILD_DIR}"
  for need in \
    "${vr_lib}" \
    "${INSTALL_ROOT}/include/lejusdk-vr" \
    "${INSTALL_ROOT}/include/lejusdk-dds-idl" \
    "${INSTALL_ROOT}/include/lejusdk-topic-pubsub" \
    "${INSTALL_ROOT}/lib/liblejusdk-dds-idl.a" \
    "${INSTALL_ROOT}/lib/liblejusdk-dds-rpc.a" \
    "${INSTALL_ROOT}/lib/libddscxx.so" \
    "${INSTALL_ROOT}/lib/libddsc.so" \
    "${PROJECT_DIR}/src/lejusdk/lejusdk-utils/include" \
    "${tp}/iceoryx/lib/libiceoryx_binding_c.a" \
    "${tp}/iceoryx/lib/libiceoryx_posh.a" \
    "${tp}/iceoryx/lib/libiceoryx_hoofs.a" \
    "${tp}/iceoryx/lib/libiceoryx_platform.a"
  do
    if [[ ! -e "${need}" ]]; then
      echo "ERROR: 开源仓库缺少文件: ${need}" >&2
      return 1
    fi
  done
  echo "用 installed/ SDK 编译 arm_traj_tracking_test (${arch})..." >&2
  "${cxx}" -std=gnu++17 -O2 -g \
    -I"${INSTALL_ROOT}/include" \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-utils/include" \
    -isystem /usr/include/eigen3 \
    -isystem "${tp}/cyclonedds-cxx-0.10.2/include/ddscxx" \
    -isystem "${tp}/cyclonedds-0.10.2/include" \
    -isystem "${iox_inc}" \
    "${TEST_CPP}" -o "${out}" \
    -Wl,-rpath,"${INSTALL_ROOT}/lib:${ROS_LIB}" \
    "${vr_lib}" -lyaml-cpp \
    "${INSTALL_ROOT}/lib/liblejusdk-dds-idl.a" \
    "${INSTALL_ROOT}/lib/liblejusdk-dds-rpc.a" \
    -L"${INSTALL_ROOT}/lib" \
    -lddscxx -lddsc \
    "${tp}/iceoryx/lib/libiceoryx_binding_c.a" \
    "${tp}/iceoryx/lib/libiceoryx_posh.a" \
    -lpthread \
    "${tp}/iceoryx/lib/libiceoryx_hoofs.a" \
    "${tp}/iceoryx/lib/libiceoryx_platform.a" \
    -lrt -lacl -latomic
  cp -f "${TEST_YAML}" "${BUILD_DIR}/arm_traj_tracking.yaml"
  echo "${out}"
}

build_test_closed() {
  local arch tp cxx out vr_lib
  case "$(uname -m)" in
    aarch64|arm64) arch="aarch64" ;;
    x86_64) arch="x86_64" ;;
    *)
      echo "ERROR: unsupported architecture: $(uname -m)" >&2
      return 1
      ;;
  esac
  tp="${PROJECT_DIR}/src/lejusdk/3rd_party/${arch}"
  cxx="$(command -v g++-11 || command -v g++)"
  vr_lib="${BUILD_DIR}/src/lejusdk/lejusdk-vr/liblejusdk-vr.so"
  out="${BUILD_DIR}/arm_traj_tracking_test"
  echo "Compiling arm_traj_tracking_test from the closed source tree..." >&2
  "${cxx}" -std=gnu++17 -O2 -g \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-vr/include" \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-vr/src" \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-comm-dds/lejusdk-dds-idl/include" \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-comm-dds/lejusdk-topic-pubsub/include" \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc/include" \
    -I"${PROJECT_DIR}/src/lejusdk/lejusdk-utils/include" \
    -isystem /usr/include/eigen3 \
    -isystem "${tp}/cyclonedds-cxx-0.10.2/include/ddscxx" \
    -isystem "${tp}/cyclonedds-0.10.2/include" \
    -isystem "${tp}/iceoryx/include/iceoryx/v" \
    "${TEST_CPP}" -o "${out}" \
    -Wl,-rpath,"${BUILD_DIR}/src/lejusdk/lejusdk-vr:${tp}/cyclonedds-cxx-0.10.2/lib:${tp}/cyclonedds-0.10.2/lib" \
    "${vr_lib}" -lyaml-cpp \
    "${tp}/cyclonedds-cxx-0.10.2/lib/libddscxx.so.0.10.2" \
    "${tp}/cyclonedds-0.10.2/lib/libddsc.so.0.10.2" \
    "${BUILD_DIR}/src/lejusdk/lejusdk-comm-dds/lejusdk-dds-idl/liblejusdk-dds-idl.a" \
    "${BUILD_DIR}/src/lejusdk/lejusdk-comm-dds/lejusdk-dds-rpc/liblejusdk-dds-rpc.a" \
    "${tp}/iceoryx/lib/libiceoryx_binding_c.a" \
    "${tp}/iceoryx/lib/libiceoryx_posh.a" \
    -lpthread \
    "${tp}/iceoryx/lib/libiceoryx_hoofs.a" \
    "${tp}/iceoryx/lib/libiceoryx_platform.a" \
    -lrt -lacl -latomic \
    "${tp}/cyclonedds-0.10.2/lib/libddsc.so.0.10.2"
  cp -f "${TEST_YAML}" "${BUILD_DIR}/arm_traj_tracking.yaml"
  echo "${out}"
}

echo "Building arm_traj_tracking_test..."
if [[ "${OPENSOURCE}" -eq 1 ]]; then
  TEST_BIN="$(build_test_opensource)"
else
  TEST_BIN="$(build_test_closed)"
fi

LAUNCH_HINT="bash src/leju_launch/scripts/launch_real.sh --urdf-path \"${URDF_PATH}\""
if [[ ! -f "${PROJECT_DIR}/src/leju_launch/scripts/launch_real.sh" ]]; then
  LAUNCH_HINT="启动 installed/bin/run_rl_controller 时加上 -u \"${URDF_PATH}\""
elif ! grep -q -- '--urdf-path' "${PROJECT_DIR}/src/leju_launch/scripts/launch_real.sh"; then
  LAUNCH_HINT="启动 run_rl_controller 时加上 -u \"${URDF_PATH}\""
fi

if [[ "${PINOCCHIO_OK}" -eq 1 ]]; then
  PINOCCHIO_NOTE="控制器已链上 Pinocchio：
  ${pinocchio_link}"
else
  PINOCCHIO_NOTE="控制器没有链 libpinocchio。测试可以跑，但手臂前馈力矩仍是 0。
替换 installed/bin/run_rl_controller 之后再启动，日志里要有 ArmTorqueController initialized。"
fi

cat <<EOF

测试程序已编译。机器人没有被启动。
${PINOCCHIO_NOTE}
测试: ${TEST_BIN}

先停掉正在跑的控制器，再带 URDF 启动。不带 URDF 时前馈力矩仍是 0：
  cd "${PROJECT_DIR}"
  source installed/setup.bash
  export LD_LIBRARY_PATH=${ROS_LIB}:\${LD_LIBRARY_PATH}
  ${LAUNCH_HINT}

然后：
  ROBOT_VERSION=17 ${TEST_BIN}
步进角和保持时间改 ${TEST_YAML}。
EOF
