add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint
  fullbody_inverse_kinematics_solver
  prioritized_qp
  #prioritized_inverse_kinematics_solver
  cpp_filters
  #ik_constraint_joint_limit_table
  hrpsys_ros_bridge
  joint_limit_table
  )
find_package(choreonoid REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  $<TARGET_PROPERTY:OsqpEigen::OsqpEigen,INTERFACE_INCLUDE_DIRECTORIES>
  )
link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(AutoStabilizer SHARED
  AutoStabilizer.cpp
  AutoStabilizerService_impl.cpp
  LegCoordsGenerator.cpp
  FootStepGenerator.cpp
  RefToGenFrameConverter.cpp
  ActToGenFrameConverter.cpp
  ExternalForceHandler.cpp
  Stabilizer.cpp
  ImpedanceController.cpp
  FullbodyIKSolver.cpp
  LegManualController.cpp
  CmdVelGenerator.cpp
  MathUtil.cpp
  )
target_link_libraries(AutoStabilizer
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  RobotHardwareServiceSkel RobotHardwareServiceStub
  #OsqpEigen::OsqpEigen # -isystemでこのワークスペースのdevel/includeがinclude_directoriesに入るのだが、優先順位末尾に入ってしまう
  #$<TARGET_PROPERTY:OsqpEigen::OsqpEigen,INTERFACE_LINK_LIBRARIES> # なくても良い?
  )
set_target_properties(AutoStabilizer PROPERTIES PREFIX "")
add_dependencies(AutoStabilizer RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild2_genidl

add_executable(AutoStabilizerComp AutoStabilizerComp.cpp)
target_link_libraries(AutoStabilizerComp AutoStabilizer)

install(TARGETS AutoStabilizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS AutoStabilizer
  LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib
)
