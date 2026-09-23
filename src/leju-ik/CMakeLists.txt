cmake_minimum_required(VERSION 3.16)
project(leju-ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# ── Catkin support (x86) ──
find_package(catkin QUIET COMPONENTS lejusdk-utils)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS lejusdk-utils
    DEPENDS EIGEN3
  )
endif()

find_package(drake CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

set(SOURCES
  src/Quest3ArmInfoTransformer.cpp
  src/QuestArmRobotMapper.cpp
  src/TwoStageTorsoIK.cpp
  src/ArmLengthMeasurement.cpp
  src/Quest3IkAPI.cpp
  src/BaseIKSolver.cpp
  src/OneStageIKEndEffector.cpp
  src/HandSmoother.cpp
  src/IncrementalControlModule.cpp
  src/Quest3IkIncrementalAPI.cpp
  src/ArmAbsoluteIK.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${EIGEN3_INCLUDE_DIRS}
)
if(_USING_CATKIN)
  target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
endif()

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  drake::drake
  yaml-cpp
)
if(_USING_CATKIN)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} lejusdk-utils)
endif()

# Keep the portable package RPATH from the root project. Drake is an external
# target-architecture runtime dependency and must not leak a build-machine path
# into the install tree.
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_RPATH "${CMAKE_BUILD_RPATH}")

# Install
if(_USING_CATKIN)
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
  )
  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
  )
  install(DIRECTORY include/nlohmann/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/nlohmann
    FILES_MATCHING PATTERN "*.hpp"
  )
else()
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
  )
  install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
  )
  install(DIRECTORY include/nlohmann/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/nlohmann
    FILES_MATCHING PATTERN "*.hpp"
  )
endif()

# Tests
option(BUILD_LEJU_IK_TESTS "Build leju-ik tests" OFF)
if(BUILD_LEJU_IK_TESTS)
  add_subdirectory(test)
endif()
