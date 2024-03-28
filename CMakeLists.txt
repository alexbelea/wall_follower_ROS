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

add_executable(active_follower src/active_follower.cpp)
target_link_libraries(active_follower ${catkin_LIBRARIES})
