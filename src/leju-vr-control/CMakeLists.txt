cmake_minimum_required(VERSION 3.16)
project(leju-vr-control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  lejusdk-lowlevel
  lejusdk-vr
  leju-ik
  lejusdk-utils
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(drake CONFIG QUIET)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS lejusdk-lowlevel lejusdk-vr leju-ik lejusdk-utils
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCES
  src/arm_ctrl_mode_fsm.cpp
  src/quest_to_ik_converter.cpp
  src/head_solver.cpp
  src/quest_vr_fsm.cpp
  src/waist_teleop_controller.cpp
  src/quest_vr_control_node.cpp
)

add_executable(quest_vr_control_node ${SOURCES})

target_include_directories(quest_vr_control_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(quest_vr_control_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)
if(drake_FOUND)
  target_link_libraries(quest_vr_control_node drake::drake)
endif()

install(TARGETS quest_vr_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
