cmake_minimum_required(VERSION 3.10)
project(leju-remote)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  lejusdk-vr
)

find_package(Protobuf REQUIRED)

catkin_package(
  CATKIN_DEPENDS lejusdk-vr
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/protos_c
)

# Pre-generated protobuf C++ sources (copied from kuavo-ros-control noitom_hi5_hand_udp_python/protos_c)
set(PROTO_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/protos_c/hand_pose.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/protos_c/robot_info.pb.cc
)

add_executable(quest_udp_to_dds_node
  src/quest_udp_to_dds_node.cpp
  ${PROTO_SRCS}
)

add_dependencies(quest_udp_to_dds_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(quest_udp_to_dds_node
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

install(TARGETS quest_udp_to_dds_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
