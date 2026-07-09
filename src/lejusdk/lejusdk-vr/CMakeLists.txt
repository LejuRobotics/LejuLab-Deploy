cmake_minimum_required(VERSION 3.0.2)
project(lejusdk-vr)

find_package(catkin REQUIRED COMPONENTS lejusdk-utils)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# Import pre-built library
# Note: cyclonedds libraries are in installed/lib and will be found at runtime
add_library(lejusdk-vr SHARED IMPORTED GLOBAL)
set_target_properties(lejusdk-vr PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/liblejusdk-vr.so
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
  IMPORTED_NO_SONAME TRUE
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lejusdk-vr
  CATKIN_DEPENDS lejusdk-utils
  DEPENDS EIGEN3
  CFG_EXTRAS
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

install(FILES lib/liblejusdk-vr.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
