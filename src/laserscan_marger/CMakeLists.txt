cmake_minimum_required(VERSION 3.8)
project(laserscan_marger)


# set(CMAKE_BUILD_TYPE Debug)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(tinyxml2_vendor REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(laserscan_marger SHARED
  src/laserscan_marger.cpp
)

rclcpp_components_register_nodes(laserscan_marger "laserscan_marger::LaserScanMarger")

ament_auto_add_executable(laserscan_marger_node
  src/laserscan_marger_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
