set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  auto_stabilizer_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS})

add_library(AutoStabilizerROSBridge SHARED AutoStabilizerROSBridge.cpp)
target_link_libraries(AutoStabilizerROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES})
set_target_properties(AutoStabilizerROSBridge PROPERTIES PREFIX "")

add_executable(AutoStabilizerROSBridgeComp AutoStabilizerROSBridgeComp.cpp)
target_link_libraries(AutoStabilizerROSBridgeComp AutoStabilizerROSBridge)

install(TARGETS AutoStabilizerROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS AutoStabilizerROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
