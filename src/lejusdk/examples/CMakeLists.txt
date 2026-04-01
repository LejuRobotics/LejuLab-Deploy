cmake_minimum_required(VERSION 3.16)
project(lejusdk_examples)

find_package(catkin REQUIRED COMPONENTS
  lejusdk-lowlevel
  lejusdk-utils
)

catkin_package()

# 示例源文件路径
set(EXAMPLE_SOURCES
    src/hw_state_subscriber.cpp
    src/joint_state_subscriber.cpp
    src/imu_data_subscriber.cpp
    src/joint_cmd_publisher.cpp
    src/joint_cmd_subscriber.cpp
    src/kuavo_control_example.cpp
)

# 为每个示例文件创建可执行程序
foreach(SOURCE_FILE ${EXAMPLE_SOURCES})
    get_filename_component(EXEC_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXEC_NAME} ${SOURCE_FILE})
    
    # 包含头文件目录
    target_include_directories(${EXEC_NAME} PRIVATE
        ${catkin_INCLUDE_DIRS}
    )
    
    # 链接库
    target_link_libraries(${EXEC_NAME}
        ${catkin_LIBRARIES}
        pthread
    )
endforeach()