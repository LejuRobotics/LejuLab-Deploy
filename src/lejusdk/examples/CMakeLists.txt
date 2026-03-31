cmake_minimum_required(VERSION 3.16)
project(lejusdk_examples)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(catkin REQUIRED COMPONENTS
  lejusdk-lowlevel
  lejusdk-vr
  lejusdk-utils
)

catkin_package()

# lejusdk-lowlevel 示例
set(LOWLEVEL_EXAMPLES
    src/hw_state_subscriber.cpp
    src/joint_state_subscriber.cpp
    src/imu_data_subscriber.cpp
    src/joint_cmd_publisher.cpp
    src/joint_cmd_subscriber.cpp
    src/kuavo_control_example.cpp
)

# lejusdk-vr 示例
set(VR_EXAMPLES
    src/lejusdk-vr/01_basic_init.cpp
    src/lejusdk-vr/02_kuavo_arm.cpp
    src/lejusdk-vr/03_velocity_cmd.cpp
    src/lejusdk-vr/04_get_state.cpp
    src/lejusdk-vr/05_head_control.cpp
    src/lejusdk-vr/06_waist_control.cpp
    src/lejusdk-vr/07_roban_arm.cpp
    src/lejusdk-vr/08_controller_cli.cpp
)

# 为 lowlevel 示例创建可执行程序
foreach(SOURCE_FILE ${LOWLEVEL_EXAMPLES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    target_include_directories(${EXEC_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${EXEC_NAME} ${catkin_LIBRARIES} pthread)
endforeach()

# 为 vr 示例创建可执行程序
foreach(SOURCE_FILE ${VR_EXAMPLES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    set(EXEC_NAME "vr_${EXEC_NAME}")
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    target_include_directories(${EXEC_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${EXEC_NAME} ${catkin_LIBRARIES} pthread)
endforeach()