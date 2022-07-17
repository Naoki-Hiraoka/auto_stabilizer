cmake_minimum_required(VERSION 3.0.2)
project(auto_stabilizer_choreonoid_sample)

find_package(catkin REQUIRED COMPONENTS
  hrpsys_choreonoid
  hrpsys_choreonoid_tutorials
  jvrc_models)

catkin_package()

configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_RH_FLAT.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_RH_FLAT.cnoid @ONLY)
