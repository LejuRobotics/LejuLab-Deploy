cmake_minimum_required(VERSION 3.10)
project(leju-remote)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS
  lejusdk-vr
)
set(_USING_CATKIN ${catkin_FOUND})

find_package(Protobuf REQUIRED)

if(_USING_CATKIN)
  catkin_package(
    CATKIN_DEPENDS lejusdk-vr
  )
endif()

# 预生成的 protobuf 文件按格式分两套：
#   protos_c_aarch64/ → protoc 3.21 格式 (port_def.inc), 默认，3.7+ 用
#   protos_c_x86/     → protoc 3.6  格式 (stubs/common.h), 3.6.x 系统用
#
# 自动检测：libprotobuf 版本 ≤ 3.6 → x86, 否则 → aarch64
# 手动覆盖：cmake .. -DPROTO_FORMAT=3.6 或 -DPROTO_FORMAT=3.21
set(PROTO_FORMAT "auto" CACHE STRING "Protobuf format: auto, 3.6, 3.21")
if(PROTO_FORMAT STREQUAL "3.6")
  set(PROTO_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protos_c_x86")
elseif(PROTO_FORMAT STREQUAL "3.21")
  set(PROTO_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protos_c_aarch64")
else()
  if(Protobuf_VERSION VERSION_LESS_EQUAL "3.6.99")
    set(PROTO_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protos_c_x86")
  else()
    set(PROTO_ARCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protos_c_aarch64")
  endif()
endif()
message(STATUS "Protobuf format: ${PROTO_FORMAT} (libprotobuf=${Protobuf_VERSION}) → ${PROTO_ARCH_DIR}")

include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${PROTO_ARCH_DIR}
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

# Pre-generated protobuf C++ sources
set(PROTO_SRCS
  ${PROTO_ARCH_DIR}/hand_pose.pb.cc
  ${PROTO_ARCH_DIR}/hand_wrench_srv.pb.cc
  ${PROTO_ARCH_DIR}/robot_info.pb.cc
)

add_executable(quest_udp_to_dds_node
  src/quest_udp_to_dds_node.cpp
  ${PROTO_SRCS}
)

if(_USING_CATKIN)
  add_dependencies(quest_udp_to_dds_node
    ${catkin_EXPORTED_TARGETS}
  )
endif()

target_link_libraries(quest_udp_to_dds_node
  ${Protobuf_LIBRARIES}
)
if(_USING_CATKIN)
  target_link_libraries(quest_udp_to_dds_node ${catkin_LIBRARIES})
else()
  target_link_libraries(quest_udp_to_dds_node lejusdk-vr)
endif()

if(_USING_CATKIN)
  install(TARGETS quest_udp_to_dds_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  install(TARGETS quest_udp_to_dds_node
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()
