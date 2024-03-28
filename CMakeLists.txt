cmake_minimum_required(VERSION 2.8.3)
project(package_template)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(YOUR_NODE_NAME src/YOUR_NODE_SOURCE.cpp)
target_link_libraries(YOUR_NODE_NAME ${catkin_LIBRARIES})
