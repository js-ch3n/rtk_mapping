cmake_minimum_required(VERSION 2.8.3)
project(gnss_conversion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
)

add_executable(gnss_conversion_node src/gnss_conversion_node.cpp)
target_link_libraries(gnss_conversion_node ${catkin_LIBRARIES})

add_executable(gnss_conversion_4dof_node src/gnss_conversion_4dof_node.cpp)
target_link_libraries(gnss_conversion_4dof_node ${catkin_LIBRARIES})
