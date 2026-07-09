cmake_minimum_required(VERSION 3.16)

find_package(catkin REQUIRED COMPONENTS leju-ik lejusdk-utils)
find_package(drake CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test_leju_ik test_leju_ik.cpp)
target_link_libraries(test_leju_ik
  leju-ik
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  drake::drake
)
target_include_directories(test_leju_ik PRIVATE
  ${catkin_INCLUDE_DIRS}
)
