cmake_minimum_required(VERSION 3.0.2)
project(lejusdk-topic-pubsub)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET)
set(_USING_CATKIN ${catkin_FOUND})

# iceoryx + CycloneDDS
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rd_party)
include(${THIRD_PARTY_DIR}/arch_select.cmake)
set(ICEORYX_DIR ${THIRD_PARTY_ARCH_DIR}/iceoryx)
set(CYCLONEDDS_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-0.10.2)
set(CYCLONEDDS_CXX_DIR ${THIRD_PARTY_ARCH_DIR}/cyclonedds-cxx-0.10.2)
list(APPEND CMAKE_PREFIX_PATH ${ICEORYX_DIR} ${CYCLONEDDS_DIR} ${CYCLONEDDS_CXX_DIR})

find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

# Header-only INTERFACE library (no .cpp sources)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(_USING_CATKIN)
  get_target_property(_cxx_inc CycloneDDS-CXX::ddscxx INTERFACE_INCLUDE_DIRECTORIES)
  catkin_package(
    INCLUDE_DIRS include ${_cxx_inc}
  )
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
