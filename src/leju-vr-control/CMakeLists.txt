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
)
if(drake_FOUND)
  target_link_libraries(quest_vr_control_node drake::drake)
endif()

install(TARGETS quest_vr_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- quest_vr_abs_control_node: Roban/biped_s17 绝对式VR控制节点 ----
if(drake_FOUND)
  add_executable(quest_vr_abs_control_node
    src/quest_vr_abs_control_node.cpp
    src/quest_to_ik_converter.cpp
    src/quest_vr_abs_fsm.cpp
    src/head_solver.cpp
  )

  target_include_directories(quest_vr_abs_control_node PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_link_libraries(quest_vr_abs_control_node
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    drake::drake
  )

  install(TARGETS quest_vr_abs_control_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# ---- test_arm_ik_node: 合成VR输入的独立IK测试节点 ----
if(drake_FOUND)
  add_executable(test_arm_ik_node
    test/test_arm_ik_node.cpp
  )

  target_include_directories(test_arm_ik_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_link_libraries(test_arm_ik_node
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    drake::drake
  )

  install(TARGETS test_arm_ik_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
