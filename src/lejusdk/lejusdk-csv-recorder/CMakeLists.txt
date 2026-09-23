cmake_minimum_required(VERSION 3.16)
project(lejusdk-csv-recorder CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# iceoryx + CycloneDDS (和 lejusdk-lowlevel 走相同的依赖解析)
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party)
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
find_package(Threads REQUIRED)

add_executable(lejusdk-csv-recorder src/main.cpp)

target_link_libraries(lejusdk-csv-recorder
    PRIVATE
    lejusdk-dds-idl
    lejusdk-topic-pubsub
    CycloneDDS-CXX::ddscxx
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    Threads::Threads
)

include(GNUInstallDirs)
install(TARGETS lejusdk-csv-recorder
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
