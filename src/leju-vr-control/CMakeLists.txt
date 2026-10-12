cmake_minimum_required(VERSION 3.16)
project(leju-vr-control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-vr leju-ik lejusdk-utils)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS lejusdk-lowlevel lejusdk-vr leju-ik lejusdk-utils
  )
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(drake CONFIG QUIET)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

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
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(quest_vr_control_node
  Eigen3::Eigen
  yaml-cpp
)
if(drake_FOUND)
  target_link_libraries(quest_vr_control_node drake::drake)
endif()
if(_USING_CATKIN)
  target_link_libraries(quest_vr_control_node ${catkin_LIBRARIES})
else()
  target_link_libraries(quest_vr_control_node lejusdk-lowlevel lejusdk-vr leju-ik lejusdk-utils)
endif()

if(_USING_CATKIN)
  install(TARGETS quest_vr_control_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
  install(TARGETS quest_vr_control_node RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ---- quest_vr_abs_control_node: Roban 绝对式VR控制节点 ----
if(drake_FOUND)
  add_executable(quest_vr_abs_control_node
    src/quest_vr_abs_control_node.cpp
    src/quest_vr_abs_fsm.cpp
    src/quest_to_ik_converter.cpp
    src/head_solver.cpp
  )

  target_include_directories(quest_vr_abs_control_node PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_compile_definitions(quest_vr_abs_control_node PRIVATE
    LEJU_ASSETS_URDF_PATH="${CMAKE_SOURCE_DIR}/src/leju_assets/models/biped_s17/urdf/drake/biped_v3_arm.urdf"
  )

  target_link_libraries(quest_vr_abs_control_node
    Eigen3::Eigen
    yaml-cpp
    drake::drake
  )
  if(_USING_CATKIN)
    target_link_libraries(quest_vr_abs_control_node ${catkin_LIBRARIES})
  else()
    target_link_libraries(quest_vr_abs_control_node lejusdk-vr lejusdk-lowlevel leju-ik lejusdk-utils)
  endif()

  if(_USING_CATKIN)
    install(TARGETS quest_vr_abs_control_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  else()
    install(TARGETS quest_vr_abs_control_node RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

# ---- test_arm_ik_node ----
if(drake_FOUND)
  add_executable(test_arm_ik_node
    test/test_arm_ik_node.cpp
  )

  target_include_directories(test_arm_ik_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
  )

  target_compile_definitions(test_arm_ik_node PRIVATE
    LEJU_ASSETS_URDF_PATH="${CMAKE_SOURCE_DIR}/src/leju_assets/models/biped_s17/urdf/drake/biped_v3_arm.urdf"
  )

  target_link_libraries(test_arm_ik_node
    Eigen3::Eigen
    drake::drake
  )
  if(_USING_CATKIN)
    target_link_libraries(test_arm_ik_node ${catkin_LIBRARIES})
  else()
    target_link_libraries(test_arm_ik_node leju-ik)
  endif()

  if(_USING_CATKIN)
    install(TARGETS test_arm_ik_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  else()
    install(TARGETS test_arm_ik_node RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endif()

if(_USING_CATKIN)
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
  )
else()
  install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
  )
endif()
