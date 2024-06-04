cmake_minimum_required(VERSION 3.0.2)
project(auto_stabilizer_ros_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED rtmbuild2)

rtmbuild2_init()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auto_stabilizer_tools
#  CATKIN_DEPENDS auto_stabilizer_msgs roscpp rtmbuild2
#  DEPENDS system_lib
)

add_subdirectory(rtc)
