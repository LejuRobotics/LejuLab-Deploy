cmake_minimum_required(VERSION 3.0.2)
project(lejusdk-utils)

include(GNUInstallDirs)

# ── Always create INTERFACE target (needed by pure CMake add_subdirectory builds) ──
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# ── Catkin support (x86 catkin workspace build) ──
find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES  # header-only, 无 .so
  )
endif()

if(catkin_FOUND)
    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
else()
    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
endif()
