cmake_minimum_required(VERSION 3.16)
project(leju-audio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

# iceoryx + CycloneDDS（IDL/pubsub 依赖需要）
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lejusdk/3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

# ============================================================================
# 音频依赖自动检测/安装
#   - ALSA 开发库 (libasound2-dev): 采集/播放必须链接，缺失则 find_package 直接失败
#   - sox: 运行时音频工具，缺失仅告警（不阻断构建）
# 全新环境（尤其板载 aarch64 首次 cmake）若缺依赖则自动 apt 安装，避免配置失败。
# 仅本机构建（非交叉）且有 apt-get 时尝试；交叉编译走 sysroot，不在此处理。
# ============================================================================
function(_leju_apt_install pkg)
  find_program(_LEJU_APT apt-get)
  if(NOT _LEJU_APT)
    message(WARNING "leju-audio: 未找到 apt-get，无法自动安装 ${pkg}，请手动安装")
    return()
  endif()
  execute_process(COMMAND id -u OUTPUT_VARIABLE _uid OUTPUT_STRIP_TRAILING_WHITESPACE)
  message(STATUS "leju-audio: 自动安装缺失依赖 ${pkg} ...")
  if(_uid STREQUAL "0")
    execute_process(COMMAND ${_LEJU_APT} install -y ${pkg} RESULT_VARIABLE _rc)
  else()
    find_program(_LEJU_SUDO sudo)
    if(NOT _LEJU_SUDO)
      message(WARNING "leju-audio: 非 root 且无 sudo，无法自动安装 ${pkg}，请手动: apt-get install -y ${pkg}")
      return()
    endif()
    # -n 非交互：无免密 sudo 时快速失败而非卡在密码提示
    execute_process(COMMAND ${_LEJU_SUDO} -n ${_LEJU_APT} install -y ${pkg} RESULT_VARIABLE _rc)
  endif()
  if(NOT _rc EQUAL 0)
    message(WARNING "leju-audio: 自动安装 ${pkg} 失败(rc=${_rc})，请手动: sudo apt-get install -y ${pkg}")
  endif()
endfunction()

# --- ALSA（必需，链接库）---
find_package(ALSA QUIET)
if(NOT ALSA_FOUND AND NOT CMAKE_CROSSCOMPILING)
  _leju_apt_install(libasound2-dev)
  unset(ALSA_LIBRARY CACHE)
  unset(ALSA_INCLUDE_DIR CACHE)
  find_package(ALSA QUIET)  # 安装后重探
endif()
find_package(ALSA REQUIRED)  # 仍缺则明确报错

# --- sox（可选，运行时音频工具）---
find_program(SOX_EXECUTABLE sox)
if(NOT SOX_EXECUTABLE AND NOT CMAKE_CROSSCOMPILING)
  _leju_apt_install(sox)
  unset(SOX_EXECUTABLE CACHE)
  find_program(SOX_EXECUTABLE sox)  # 安装后重探
endif()
if(SOX_EXECUTABLE)
  message(STATUS "leju-audio: sox 已就绪: ${SOX_EXECUTABLE}")
else()
  message(WARNING "leju-audio: 未检测到 sox（运行时音频工具，可用 ffmpeg 代替）; 需要请手动: sudo apt-get install -y sox")
endif()

find_package(CycloneDDS-CXX REQUIRED)

# ── 采集侧（capture）────────────────────────────────────────────────
add_library(leju_audio_capture STATIC
  src/mic_capture.cpp
)
target_include_directories(leju_audio_capture PUBLIC src)
target_link_libraries(leju_audio_capture PUBLIC ${ALSA_LIBRARIES})

# 单测（不依赖 DDS）
add_executable(mic_capture_test src/mic_capture_test.cpp)
target_link_libraries(mic_capture_test PRIVATE leju_audio_capture)

# 发布节点
add_executable(micphone_to_dds_node src/micphone_to_dds_node.cpp)
target_link_libraries(micphone_to_dds_node
  PRIVATE
    leju_audio_capture
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    CycloneDDS-CXX::ddscxx
)
install(TARGETS micphone_to_dds_node
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ── 播放侧（playback）──────────────────────────────────────────────
# ALSA 播放封装 + ffmpeg 解码（不依赖 DDS）
add_library(leju_audio_playback STATIC
  src/speaker_output.cpp
  src/audio_decoder.cpp
)
target_include_directories(leju_audio_playback PUBLIC src)
target_link_libraries(leju_audio_playback PUBLIC ${ALSA_LIBRARIES})

# 单测（不依赖 DDS）
add_executable(speaker_output_test src/speaker_output_test.cpp)
target_link_libraries(speaker_output_test PRIVATE leju_audio_playback)

# 播放节点（RPC play_file + DDS PCM 流 + stop）
add_executable(audio_player_node src/audio_player_node.cpp)
target_link_libraries(audio_player_node
  PRIVATE
    leju_audio_playback
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    lejusdk-dds-rpc
    CycloneDDS-CXX::ddscxx
)

# 命令行调试工具
add_executable(play_audio_cli src/play_audio_cli.cpp)
target_link_libraries(play_audio_cli
  PRIVATE
    lejusdk-dds-rpc
    CycloneDDS-CXX::ddscxx
)

# 诊断工具：合成 PCM 推流 + stop（验证 DDS 流输入路径）
add_executable(audio_stream_probe src/audio_stream_probe.cpp)
target_link_libraries(audio_stream_probe
  PRIVATE
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    CycloneDDS-CXX::ddscxx
)

install(TARGETS audio_player_node play_audio_cli audio_stream_probe
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
