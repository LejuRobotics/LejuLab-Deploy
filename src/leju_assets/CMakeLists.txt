cmake_minimum_required(VERSION 3.0.2)
project(leju_assets)

include(GNUInstallDirs)

# ── Catkin support ──
find_package(catkin QUIET)
set(_USING_CATKIN ${catkin_FOUND})

if(_USING_CATKIN)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
endif()

add_library(${PROJECT_NAME} SHARED src/package_path.cc)
target_compile_definitions(${PROJECT_NAME} PRIVATE PROJECT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

if(_USING_CATKIN)
  target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
else()
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  )
endif()
