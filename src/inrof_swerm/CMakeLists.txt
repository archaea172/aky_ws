cmake_minimum_required(VERSION 3.8)
project(inrof_swerm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_library(inrof_swerm_core
  src/core/boid_core.cpp
  src/core/follower_core.cpp
  src/core/sheep_core.cpp
)

target_link_libraries(inrof_swerm_core
  Eigen3::Eigen
)

add_library(inrof_swerm_ros
  src/ros/boid_node.cpp
  src/ros/follower_node.cpp
)

ament_target_dependencies(inrof_swerm_ros
  rclcpp
  nav_msgs
  geometry_msgs
)

target_link_libraries(inrof_swerm_ros
  Eigen3::Eigen
  inrof_swerm_core
)

target_link_libraries(inrof_swerm_ros
  Eigen3::Eigen
)

add_executable(boid_node
  src/boid_main.cpp
)

target_link_libraries(boid_node
  inrof_swerm_ros
)

install(
  TARGETS boid_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(follower_node
  src/follower_main.cpp
)

target_link_libraries(follower_node
  inrof_swerm_ros
)

install(
  TARGETS follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
