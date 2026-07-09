cmake_minimum_required(VERSION 3.0.2)
project(leju-rl-controller)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type to Release (CMake default)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS lejusdk-lowlevel lejusdk-utils lejusdk-vr)

## System dependencies are found with CMake's conventions
# find_package(OpenVINO REQUIRED HINTS 3rd_party/openvino_toolkit/runtime COMPONENTS Runtime Threading) # you have to install tbb?
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# ONNX Runtime (自动检测 3rd_party 库是否存在)
set(ONNXRUNTIME_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/onnxruntime_toolkit/onnxruntime-linux-x64-1.18.0/lib/libonnxruntime.so")
message(STATUS "Checking ONNX Runtime library: ${ONNXRUNTIME_LIB_PATH}")

if(EXISTS "${ONNXRUNTIME_LIB_PATH}")
  set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/onnxruntime_toolkit/onnxruntime-linux-x64-1.18.0")
  set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_ROOT}/include")
  set(ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

  # 设置 CMake 变量供后续条件判断使用
  set(USE_ONNXRUNTIME TRUE)
  # 添加编译器宏定义
  add_compile_definitions(USE_ONNXRUNTIME)
  message(STATUS "ONNX Runtime backend: enabled (auto-detected)")
  message(STATUS "  Include: ${ONNXRUNTIME_INCLUDE_DIRS}")
  message(STATUS "  Library: ${ONNXRUNTIME_LIBRARIES}")
else()
  message(STATUS "ONNX Runtime backend: disabled (library not found in 3rd_party)")
  message(WARNING "ONNX Runtime library not found at: ${ONNXRUNTIME_LIB_PATH}")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rl_controller_framework
  CATKIN_DEPENDS lejusdk-lowlevel lejusdk-utils lejusdk-vr
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(INC_DIRS
  include
  examples/include
  ${catkin_INCLUDE_DIRS}
)

# ONNX Runtime include
if(USE_ONNXRUNTIME)
  list(APPEND INC_DIRS ${ONNXRUNTIME_INCLUDE_DIRS})
endif()

include_directories(${INC_DIRS})

## Declare a C++ library
add_library(rl_controller_framework
  src/controllers/controller_base.cpp
  src/controllers/controller_manager.cpp
  src/controllers/generic_rl_controller.cpp
  src/robot_data.cpp
  src/joy_handler.cpp
  src/velocity_manager.cpp
  src/motion/motion_trajectory.cpp
  src/inference/openvino_model.cpp
  src/inference/onnxruntime_model.cpp
  src/trajectory/minimum_jerk_interpolator.cpp
  src/trajectory/velocity_limited_generator.cpp
  src/rl/arm_torque_controller.cpp
  src/rl/multi_mode_arm_controller.cpp
  src/rl/waist_controller.cpp
  src/utils/uri_path_resolver.cpp
)
target_link_libraries(rl_controller_framework
  ${catkin_LIBRARIES}
  openvino::runtime
  yaml-cpp
  Eigen3::Eigen
  pinocchio::pinocchio
)

# ONNX Runtime support
if(USE_ONNXRUNTIME)
  target_include_directories(rl_controller_framework PUBLIC ${ONNXRUNTIME_INCLUDE_DIRS})
  target_link_libraries(rl_controller_framework ${ONNXRUNTIME_LIBRARIES})
endif()

# Build examples subdirectory
add_subdirectory(examples)

## Declare a C++ executable
add_executable(run_rl_controller src/main.cpp)
target_link_libraries(run_rl_controller
  ${catkin_LIBRARIES}
  rl_controller_framework
  rl_demo_controllers
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

set(CATKIN_ENABLE_TESTING ON)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Test: MotionTrajectory
  catkin_add_gtest(test_motion_trajectory tests/motion/test_motion_trajectory.cpp)
  if(TARGET test_motion_trajectory)
    target_link_libraries(test_motion_trajectory
      rl_controller_framework
      ${catkin_LIBRARIES}
      Eigen3::Eigen
    )
  endif()

  # Test: RobotData
  catkin_add_gtest(test_robot_data tests/test_robot_data.cpp)
  if(TARGET test_robot_data)
    target_link_libraries(test_robot_data
      rl_controller_framework
      ${catkin_LIBRARIES}
    )
  endif()

  # Test: MinimumJerkInterpolator
  catkin_add_gtest(test_minimum_jerk_interpolator tests/trajectory/interpolator/test_minimum_jerk_interpolator.cpp)
  if(TARGET test_minimum_jerk_interpolator)
    target_link_libraries(test_minimum_jerk_interpolator
      rl_controller_framework
      ${catkin_LIBRARIES}
      Eigen3::Eigen
    )
  endif()

  # Test: VelocityLimitedGenerator
  catkin_add_gtest(test_velocity_limited_generator tests/trajectory/generator/test_velocity_limited_generator.cpp)
  if(TARGET test_velocity_limited_generator)
    target_link_libraries(test_velocity_limited_generator
      rl_controller_framework
      ${catkin_LIBRARIES}
      Eigen3::Eigen
    )
  endif()
endif()
