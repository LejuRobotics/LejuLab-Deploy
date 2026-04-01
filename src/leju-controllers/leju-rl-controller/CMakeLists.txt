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
find_package(catkin REQUIRED COMPONENTS lejusdk-lowlevel lejusdk-utils)

## System dependencies are found with CMake's conventions
# find_package(OpenVINO REQUIRED HINTS 3rd_party/openvino_toolkit/runtime COMPONENTS Runtime Threading) # you have to install tbb?
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

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
  LIBRARIES rl_controller_framework rl_demo_controllers
  CATKIN_DEPENDS lejusdk-lowlevel lejusdk-utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rl_controller_framework
  src/controllers/controller_base.cpp
  src/controllers/controller_manager.cpp
  src/controllers/generic_rl_controller.cpp
  src/robot_data.cpp
  src/joy_handler.cpp
  src/motion/motion_trajectory.cpp
  src/inference/openvino_model.cpp
)
target_link_libraries(rl_controller_framework
  ${catkin_LIBRARIES}
  openvino::runtime
  yaml-cpp
  Eigen3::Eigen
)

add_library(rl_demo_controllers
  src/controllers/rl_demo_controller.cpp
  src/controllers/rl_mimic_controller.cpp
  src/cmd_stance_calculator.cpp
  src/arm_controller.cpp
)
target_link_libraries(rl_demo_controllers
  ${catkin_LIBRARIES}
  rl_controller_framework
  openvino::runtime
  yaml-cpp
  Eigen3::Eigen
)

## Declare a C++ executable
add_executable(run_rl_controller src/main.cpp)
target_link_libraries(run_rl_controller
  ${catkin_LIBRARIES}
  rl_controller_framework
  rl_demo_controllers
)

add_executable(run_rl_demo_controller src/run_rl_demo_controller.cpp)
target_link_libraries(run_rl_demo_controller
  ${catkin_LIBRARIES}
  rl_controller_framework
  rl_demo_controllers
)

add_executable(run_rl_mimic_controller src/run_rl_mimic_controller.cpp)
target_link_libraries(run_rl_mimic_controller
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Test: MotionLoader
  catkin_add_gtest(test_motion_trajectory tests/test_motion_trajectory.cpp)
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
endif()
