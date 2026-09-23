cmake_minimum_required(VERSION 3.16)
project(leju-monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)

# iceoryx + CycloneDDS（IDL/pubsub 依赖需要）
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lejusdk/3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

find_package(CycloneDDS-CXX REQUIRED)

# 系统监控发布节点（/monitor/system_info/*）
add_executable(system_info_node src/system_info_node.cpp)
target_link_libraries(system_info_node
  PRIVATE
    lejusdk-utils
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    CycloneDDS-CXX::ddscxx
)

# 调试工具：订阅并打印全部监控话题
add_executable(system_info_echo src/system_info_echo.cpp)
target_link_libraries(system_info_echo
  PRIVATE
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    CycloneDDS-CXX::ddscxx
)

install(TARGETS system_info_node system_info_echo
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
