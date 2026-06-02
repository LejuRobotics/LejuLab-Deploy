cmake_minimum_required(VERSION 3.16)
project(leju-ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS lejusdk-utils)

find_package(drake CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# nlohmann json: 使用 include/nlohmann/json.hpp（单头文件，无需 FetchContent）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS lejusdk-utils
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

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
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  drake::drake
  yaml-cpp
)


# 设置 RPATH 确保运行时能找到 libdrake.so
get_target_property(_drake_lib drake::drake IMPORTED_LOCATION)
if(_drake_lib)
    get_filename_component(_drake_lib_dir "${_drake_lib}" DIRECTORY)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        INSTALL_RPATH "${_drake_lib_dir}"
        BUILD_RPATH   "${_drake_lib_dir}"
    )
endif()

#############
## Install ##
#############
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

# Tests (Drake 编译较重，默认关闭以加速日常构建)
option(BUILD_LEJU_IK_TESTS "Build leju-ik tests" OFF)
if(BUILD_LEJU_IK_TESTS)
  add_subdirectory(test)
endif()
