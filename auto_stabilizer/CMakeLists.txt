cmake_minimum_required(VERSION 3.0.2)
project(auto_stabilizer)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
  hrpsys_ros_bridge
  )

# catkin_python_setup()
# initialize rtmbuild (generate_messages is called)
rtmbuild2_init(hrpsys_ros_bridge rtmbuild2)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auto_stabilizer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# generate idl
#set(rtm_idlflags ${rtm_idlflags} -I${hrpsys_IDL_DIR})
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

###########
## Build ##
###########

#add_subdirectory(rtc)
