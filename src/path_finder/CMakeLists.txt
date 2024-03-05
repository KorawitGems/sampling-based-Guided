cmake_minimum_required(VERSION 3.0.2)
project(path_finder)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  occ_grid
  self_msgs_and_srvs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_finder
  CATKIN_DEPENDS occ_grid geometry_msgs tf2 tf2_ros pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/test_planner.cpp
  src/kdtree.c
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sampling_global_planner_node
  src/sampling_global_planner_node.cpp
  src/kdtree.c
)
target_link_libraries(sampling_global_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sampling_local_planner_node
  src/sampling_local_planner_node.cpp
  src/kdtree.c
)
target_link_libraries(sampling_local_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)