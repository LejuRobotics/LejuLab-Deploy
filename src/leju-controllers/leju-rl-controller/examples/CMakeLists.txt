# Examples subdirectory CMakeLists.txt
# This file is included by the parent CMakeLists.txt

## Specify additional locations of header files for examples
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Also add to parent scope include directories for targets in parent
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

## Declare examples library
add_library(rl_demo_controllers
  src/rl_demo_controller.cpp
  src/rl_mimic_controller.cpp
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

# ONNX Runtime support for examples
if(USE_ONNXRUNTIME)
  target_include_directories(rl_demo_controllers PUBLIC ${ONNXRUNTIME_INCLUDE_DIRS})
  target_link_libraries(rl_demo_controllers ${ONNXRUNTIME_LIBRARIES})
endif()

## Declare executables
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

# Add dependency to ensure framework is built first
add_dependencies(rl_demo_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
