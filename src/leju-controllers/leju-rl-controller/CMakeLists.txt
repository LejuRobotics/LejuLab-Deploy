cmake_minimum_required(VERSION 3.16)
project(leju-rl-controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET COMPONENTS lejusdk-lowlevel lejusdk-utils lejusdk-vr)
set(_USING_CATKIN ${catkin_FOUND})

# iceoryx + CycloneDDS (needed by lejusdk-lowlevel headers)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lejusdk/3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})
find_package(iceoryx_hoofs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

# ---------------------------------------------------------------------------
# Optional dependencies (QUIET — never fail the build if absent)
# ---------------------------------------------------------------------------

# Pinocchio (gravity compensation) — can be disabled with -DENABLE_PINOCCHIO=OFF
option(ENABLE_PINOCCHIO "Enable Pinocchio gravity compensation (requires pinocchio)" OFF)
if(ENABLE_PINOCCHIO)
    find_package(pinocchio QUIET)
    if(pinocchio_FOUND)
        message(STATUS "Pinocchio found: gravity compensation ENABLED")
        add_compile_definitions(USE_PINOCCHIO)
    else()
        message(STATUS "Pinocchio not found: gravity compensation DISABLED")
    endif()
else()
    message(STATUS "Pinocchio: DISABLED by ENABLE_PINOCCHIO=OFF (gravity compensation off)")
endif()

# OpenVINO (optional — x86 only)
find_package(OpenVINO QUIET)
if(OpenVINO_FOUND)
    message(STATUS "OpenVINO backend: enabled")
else()
    message(STATUS "OpenVINO backend: disabled (not found)")
endif()

# ONNX Runtime (auto-detect from 3rd_party, arch-aware)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ONNXRUNTIME_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/onnxruntime_toolkit/onnxruntime-linux-aarch64/lib/libonnxruntime.so")
else()
    set(ONNXRUNTIME_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/onnxruntime_toolkit/onnxruntime-linux-x64-1.18.0/lib/libonnxruntime.so")
endif()
if(EXISTS "${ONNXRUNTIME_LIB_PATH}")
    get_filename_component(ONNXRUNTIME_ROOT "${ONNXRUNTIME_LIB_PATH}" DIRECTORY)
    get_filename_component(ONNXRUNTIME_ROOT "${ONNXRUNTIME_ROOT}" DIRECTORY)
    set(ONNXRUNTIME_INCLUDE_DIRS "${ONNXRUNTIME_ROOT}/include")
    set(ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_LIB_PATH}")
    set(USE_ONNXRUNTIME TRUE)
    add_compile_definitions(USE_ONNXRUNTIME)
    message(STATUS "ONNX Runtime backend: enabled (auto-detected)")
    message(STATUS "  Include: ${ONNXRUNTIME_INCLUDE_DIRS}")
    message(STATUS "  Library: ${ONNXRUNTIME_LIBRARIES}")
else()
    message(STATUS "ONNX Runtime backend: disabled (library not found for ${CMAKE_SYSTEM_PROCESSOR})")
endif()

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Catkin package config ──
if(_USING_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rl_controller_framework
    CATKIN_DEPENDS lejusdk-lowlevel lejusdk-utils lejusdk-vr
  )
endif()

###########
## Build ##
###########

set(INC_DIRS
    include
    examples/include
    3rd_party/magic_enum/include
    ../../lejusdk/lejusdk-recorder/thirdparty/nlohmann-json/include
    ${ICEORYX_DIR}/include/iceoryx/v
    ${CYCLONEDDS_CXX_DIR}/include
    ${CYCLONEDDS_CXX_DIR}/include/ddscxx
    ${CMAKE_CURRENT_SOURCE_DIR}/../../leju-hardware/lejusdk-hw/src/kuavo_solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lejusdk/lejusdk-comm-dds/lejusdk-dds-idl/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lejusdk/lejusdk-comm-dds/lejusdk-topic-pubsub/include
)
if(_USING_CATKIN)
    list(APPEND INC_DIRS ${catkin_INCLUDE_DIRS})
endif()
if(USE_ONNXRUNTIME)
    list(APPEND INC_DIRS ${ONNXRUNTIME_INCLUDE_DIRS})
endif()
include_directories(${INC_DIRS})

# ---------------------------------------------------------------------------
# rl_controller_framework — core library (all dev source files)
# ---------------------------------------------------------------------------
set(RL_FRAMEWORK_SOURCES
    src/controllers/controller_base.cpp
    src/controllers/controller_manager.cpp
    src/controllers/controller_registry.cpp
    src/controllers/generic_rl_controller.cpp
    src/controllers/depth_walk_controller.cpp
    src/controllers/hold_pose_controller.cpp
    src/depth/depth_image_processor.cpp
    src/depth/depth_history_buffer.cpp
    src/depth/depth_observation_provider.cpp
    src/robot_data.cpp
    src/motion/motion_trajectory.cpp
    src/motion/tact_parser.cpp
    src/motion/bezier_interpolator.cpp
    src/motion/tact_player.cpp
    src/trajectory/minimum_jerk_interpolator.cpp
    src/trajectory/velocity_limited_generator.cpp
    src/rl/arm_torque_controller.cpp
    src/rl/multi_mode_arm_controller.cpp
    src/rl/waist_controller.cpp
    src/utils/uri_path_resolver.cpp
    src/rl_frequency_csv_logger.cpp
    src/runtime/lifecycle.cpp
    src/runtime/control_loop.cpp
    src/runtime/control_logic.cpp
    src/runtime/transport_mode_coordinator.cpp
    src/runtime/fall_stand_coordinator.cpp
    src/runtime/transport_fall_stand_scheduler.cpp
    src/runtime/input/action_trigger.cpp
    src/runtime/input/command_buffer.cpp
    src/runtime/input/trigger_buffer.cpp
    src/runtime/input/external_interface.cpp
    src/runtime/input/teleop/teleop_binding_config.cpp
    src/runtime/input/teleop/joy_teleop_adapter.cpp
    src/runtime/input/teleop/quest_teleop_adapter.cpp
    src/runtime/input/teleop/teleop_input_source.cpp
)

if(OpenVINO_FOUND)
    list(APPEND RL_FRAMEWORK_SOURCES src/inference/openvino_model.cpp)
endif()
if(USE_ONNXRUNTIME)
    list(APPEND RL_FRAMEWORK_SOURCES src/inference/onnxruntime_model.cpp)
endif()

if(_USING_CATKIN)
  link_directories(
    ${CMAKE_BINARY_DIR}/../leju-hardware/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../installed/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../devel/lib
  )
endif()

add_library(rl_controller_framework ${RL_FRAMEWORK_SOURCES})

target_link_libraries(rl_controller_framework
    yaml-cpp
    Eigen3::Eigen
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
)
if(_USING_CATKIN)
    target_link_libraries(rl_controller_framework ${catkin_LIBRARIES} kuavo_solver lejusdk-dds-idl)
else()
    target_link_libraries(rl_controller_framework lejusdk-lowlevel lejusdk-vr kuavo_solver)
endif()
if(OpenVINO_FOUND)
    target_link_libraries(rl_controller_framework openvino::runtime)
    target_compile_definitions(rl_controller_framework PUBLIC USE_OPENVINO)
endif()
if(USE_ONNXRUNTIME)
    target_include_directories(rl_controller_framework PUBLIC ${ONNXRUNTIME_INCLUDE_DIRS})
    target_link_libraries(rl_controller_framework ${ONNXRUNTIME_LIBRARIES})
endif()
if(ENABLE_PINOCCHIO AND pinocchio_FOUND)
    target_link_libraries(rl_controller_framework pinocchio::pinocchio)
endif()

# ---------------------------------------------------------------------------
# rl_demo_controllers — examples library (controllers + adapters)
# ---------------------------------------------------------------------------
set(RL_DEMO_SOURCES
    examples/src/rl_mimic_controller.cpp
    examples/src/cmd_stance_calculator.cpp
    examples/src/arm_controller.cpp
)
if(OpenVINO_FOUND)
    list(APPEND RL_DEMO_SOURCES examples/src/rl_demo_controller.cpp)
endif()

add_library(rl_demo_controllers ${RL_DEMO_SOURCES})
target_link_libraries(rl_demo_controllers
    rl_controller_framework
    yaml-cpp
    Eigen3::Eigen
)
if(_USING_CATKIN)
    target_link_libraries(rl_demo_controllers ${catkin_LIBRARIES})
else()
    target_link_libraries(rl_demo_controllers lejusdk-lowlevel)
endif()
if(OpenVINO_FOUND)
    target_link_libraries(rl_demo_controllers openvino::runtime)
endif()
if(USE_ONNXRUNTIME)
    target_include_directories(rl_demo_controllers PUBLIC ${ONNXRUNTIME_INCLUDE_DIRS})
    target_link_libraries(rl_demo_controllers ${ONNXRUNTIME_LIBRARIES})
endif()

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(run_rl_controller src/main.cpp)
target_link_libraries(run_rl_controller
    rl_controller_framework
    rl_demo_controllers
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
)
if(_USING_CATKIN)
    target_link_libraries(run_rl_controller ${catkin_LIBRARIES})
else()
    target_link_libraries(run_rl_controller lejusdk-lowlevel lejusdk-vr)
endif()

add_executable(run_rl_mimic_controller examples/src/run_rl_mimic_controller.cpp)
target_link_libraries(run_rl_mimic_controller
    rl_controller_framework
    rl_demo_controllers
)
if(_USING_CATKIN)
    target_link_libraries(run_rl_mimic_controller ${catkin_LIBRARIES})
else()
    target_link_libraries(run_rl_mimic_controller lejusdk-lowlevel)
endif()

if(OpenVINO_FOUND)
    add_executable(run_rl_demo_controller examples/src/run_rl_demo_controller.cpp)
    target_link_libraries(run_rl_demo_controller
        rl_controller_framework
        rl_demo_controllers
    )
    if(_USING_CATKIN)
        target_link_libraries(run_rl_demo_controller ${catkin_LIBRARIES})
    else()
        target_link_libraries(run_rl_demo_controller lejusdk-lowlevel)
    endif()
endif()

add_executable(controller_switch_cli tools/controller_switch_cli.cpp)
if(_USING_CATKIN)
    target_link_libraries(controller_switch_cli ${catkin_LIBRARIES})
else()
    target_link_libraries(controller_switch_cli lejusdk-vr)
endif()

add_executable(posture_height_cmd_node tools/posture_height_cmd_node.cpp)
target_link_libraries(posture_height_cmd_node
    CycloneDDS-CXX::ddscxx
)
if(_USING_CATKIN)
    target_link_libraries(posture_height_cmd_node ${catkin_LIBRARIES} lejusdk-dds-idl)
else()
    target_link_libraries(posture_height_cmd_node lejusdk-lowlevel)
endif()

#############
## Install ##
#############

set(_rl_exec_targets run_rl_controller run_rl_mimic_controller controller_switch_cli posture_height_cmd_node)
if(OpenVINO_FOUND)
    list(APPEND _rl_exec_targets run_rl_demo_controller)
endif()

if(_USING_CATKIN)
    install(TARGETS ${_rl_exec_targets}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(TARGETS rl_controller_framework rl_demo_controllers
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
else()
    install(TARGETS ${_rl_exec_targets}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(TARGETS rl_controller_framework rl_demo_controllers
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

# Install ONNX Runtime shared library (needed at runtime)
if(USE_ONNXRUNTIME AND EXISTS "${ONNXRUNTIME_LIB_PATH}")
    install(DIRECTORY "${ONNXRUNTIME_ROOT}/lib/"
            DESTINATION ${CMAKE_INSTALL_LIBDIR}
            FILES_MATCHING PATTERN "libonnxruntime*.so*")
endif()

#############
## Testing ##
#############

if(_USING_CATKIN)
    # ── Catkin tests (catkin_add_gtest) ──
    option(CATKIN_ENABLE_TESTING "Enable catkin tests" ON)
    if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)
        macro(rl_test name)
          catkin_add_gtest(${name} ${ARGN})
          if(TARGET ${name})
            target_link_libraries(${name} rl_controller_framework ${catkin_LIBRARIES})
          endif()
        endmacro()
        rl_test(test_motion_trajectory tests/motion/test_motion_trajectory.cpp)
        target_link_libraries(test_motion_trajectory Eigen3::Eigen)
        rl_test(test_robot_data tests/test_robot_data.cpp)
        rl_test(test_minimum_jerk_interpolator tests/trajectory/interpolator/test_minimum_jerk_interpolator.cpp)
        target_link_libraries(test_minimum_jerk_interpolator Eigen3::Eigen)
        rl_test(test_velocity_limited_generator tests/trajectory/generator/test_velocity_limited_generator.cpp)
        target_link_libraries(test_velocity_limited_generator Eigen3::Eigen)
        rl_test(test_action_trigger tests/runtime/test_action_trigger.cpp)
        rl_test(test_trigger_buffer tests/runtime/test_trigger_buffer.cpp)
        rl_test(test_teleop_binding tests/runtime/test_teleop_binding.cpp)
        rl_test(test_teleop_adapter tests/runtime/test_teleop_adapter.cpp)
        rl_test(test_teleop_integration tests/runtime/test_teleop_integration.cpp)
        rl_test(test_depth_pipeline tests/depth/test_depth_pipeline.cpp)
        rl_test(test_depth_actuator_contract tests/depth/test_depth_actuator_contract.cpp)
    endif()
elseif(BUILD_TESTING)
    # ── Pure CMake tests (GTest directly) ──
    enable_testing()
    find_package(GTest REQUIRED)

    macro(_rl_add_test name)
        add_executable(${name} ${ARGN})
        target_link_libraries(${name}
            rl_controller_framework
            GTest::gtest
            GTest::gtest_main
        )
        add_test(NAME ${name} COMMAND ${name})
    endmacro()

    _rl_add_test(test_robot_data tests/test_robot_data.cpp)

    _rl_add_test(test_velocity_stop_filter tests/test_velocity_stop_filter.cpp)

    # Test: MixedMotionLimiter
    _rl_add_test(test_mixed_motion_limiter tests/test_mixed_motion_limiter.cpp)

    _rl_add_test(test_motion_trajectory tests/motion/test_motion_trajectory.cpp)
    target_link_libraries(test_motion_trajectory Eigen3::Eigen)

    # Test: TactApproach（tact 播放前置接入过渡，zx/develop 新增）
    _rl_add_test(test_tact_approach tests/motion/test_tact_approach.cpp)
    target_link_libraries(test_tact_approach Eigen3::Eigen)

    _rl_add_test(test_minimum_jerk_interpolator tests/trajectory/interpolator/test_minimum_jerk_interpolator.cpp)
    target_link_libraries(test_minimum_jerk_interpolator Eigen3::Eigen)

    _rl_add_test(test_velocity_limited_generator tests/trajectory/generator/test_velocity_limited_generator.cpp)
    target_link_libraries(test_velocity_limited_generator Eigen3::Eigen)

    _rl_add_test(test_action_trigger tests/runtime/test_action_trigger.cpp)

    _rl_add_test(test_arm_command_override tests/controllers/test_arm_command_override.cpp)
    target_link_libraries(test_arm_command_override Eigen3::Eigen)

    _rl_add_test(test_trigger_buffer tests/runtime/test_trigger_buffer.cpp)

    _rl_add_test(test_teleop_binding tests/runtime/test_teleop_binding.cpp)

    _rl_add_test(test_teleop_adapter tests/runtime/test_teleop_adapter.cpp)

    _rl_add_test(test_teleop_integration tests/runtime/test_teleop_integration.cpp)

    _rl_add_test(test_teleop_config_reload tests/runtime/test_teleop_config_reload.cpp)

    # Test: ExternalInterface normalized velocity mapping and receive timeout
    _rl_add_test(test_external_interface tests/runtime/test_external_interface.cpp)

    # Tool: reload_smoketest (DDS 联调工具, 非 gtest, 不驱动机器人)
    add_executable(reload_smoketest tests/tools/reload_smoketest.cpp)
    target_link_libraries(reload_smoketest lejusdk-lowlevel rl_controller_framework)

    _rl_add_test(test_joy_trigger_layer tests/runtime/test_joy_trigger_layer.cpp)
    target_compile_definitions(test_joy_trigger_layer PRIVATE
        JOY_V17_BINDINGS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/17/teleop_bindings.yaml")

    _rl_add_test(test_joy_control_chain tests/runtime/test_joy_control_chain.cpp)
endif()
