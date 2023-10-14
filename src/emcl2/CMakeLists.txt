cmake_minimum_required(VERSION 3.8)
project(emcl2)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(emcl2_node
  src/emcl2_node.cpp
  src/LikelihoodFieldMap.cpp
  src/Scan.cpp
  src/Pose.cpp
  src/OdomModel.cpp
  src/Particle.cpp
  src/Mcl.cpp
  src/lookup_tables.cpp
)

# target_compile_options(emcl2_node PRIVATE -fsanitize=undefined)
# target_link_options(emcl2_node PRIVATE -fsanitize=undefined)

ament_auto_package()
