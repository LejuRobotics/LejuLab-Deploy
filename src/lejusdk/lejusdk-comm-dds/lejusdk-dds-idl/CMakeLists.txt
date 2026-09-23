cmake_minimum_required(VERSION 3.10)
project(lejusdk-dds-idl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
add_definitions(-fPIC)

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

find_package(iceoryx_binding_c REQUIRED)
find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

if(_USING_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
  include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

file(GLOB IDL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src-generated/*.cpp")
file(GLOB IDL_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/lejusdk-dds-idl/*.hpp")

add_library(${PROJECT_NAME} STATIC ${IDL_SOURCES} ${IDL_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROJECT_NAME}
    pthread
    CycloneDDS-CXX::ddscxx
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
