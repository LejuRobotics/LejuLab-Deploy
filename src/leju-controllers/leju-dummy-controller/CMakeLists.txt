cmake_minimum_required(VERSION 3.0.2)
project(leju-dummy-controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-utils)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES dummy_controller
   CATKIN_DEPENDS lejusdk-lowlevel lejusdk-utils
  )
endif()

find_package(yaml-cpp REQUIRED)

include_directories(include)
if(_USING_CATKIN)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

add_library(dummy_controller src/dummy_controller.cpp)
if(_USING_CATKIN)
  target_link_libraries(dummy_controller ${catkin_LIBRARIES})
else()
  target_link_libraries(dummy_controller lejusdk-lowlevel lejusdk-utils)
endif()

add_library(joint_monkey_controller src/joint_monkey_controller.cpp)
if(_USING_CATKIN)
  target_link_libraries(joint_monkey_controller ${catkin_LIBRARIES} yaml-cpp)
else()
  target_link_libraries(joint_monkey_controller lejusdk-lowlevel lejusdk-utils yaml-cpp)
endif()

add_executable(run_dummy_controller src/run_dummy_controller.cpp)
target_link_libraries(run_dummy_controller dummy_controller joint_monkey_controller)
if(_USING_CATKIN)
  target_link_libraries(run_dummy_controller ${catkin_LIBRARIES})
else()
  target_link_libraries(run_dummy_controller lejusdk-lowlevel lejusdk-utils)
endif()
