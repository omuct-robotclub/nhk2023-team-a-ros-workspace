cmake_minimum_required(VERSION 3.8)
project(emcl2)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")
# set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(Mcl_emcl2 src/Mcl.cpp)
ament_auto_add_library(ExpResetMcl2_emcl2 src/ExpResetMcl2.cpp)
ament_auto_add_library(Particle_emcl2 src/Particle.cpp)
ament_auto_add_library(OdomModel_emcl2 src/OdomModel.cpp)
ament_auto_add_library(Pose_emcl2 src/Pose.cpp)
ament_auto_add_library(Scan_emcl2 src/Scan.cpp)
ament_auto_add_library(LikelihoodFieldMap_emcl2 src/LikelihoodFieldMap.cpp)

ament_auto_add_executable(emcl2_node
  src/emcl2_node.cpp
)

ament_auto_package()
