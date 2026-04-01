cmake_minimum_required(VERSION 3.0.2)
project(lejusdk-lowlevel)

find_package(catkin REQUIRED)

add_library(lejusdk-lowlevel SHARED IMPORTED GLOBAL)
set_target_properties(lejusdk-lowlevel PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/liblejusdk-lowlevel.so
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lejusdk-lowlevel
  CATKIN_DEPENDS
  DEPENDS
  CFG_EXTRAS
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

install(FILES lib/liblejusdk-lowlevel.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
