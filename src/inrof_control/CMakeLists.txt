cmake_minimum_required(VERSION 3.8)
project(inrof_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(swerm_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(path_generate_server
  src/path_generate_server.cpp
)
target_link_libraries(path_generate_server
  Eigen3::Eigen
)
ament_target_dependencies(path_generate_server
  rclcpp
  swerm_msgs
  nav_msgs
  geometry_msgs
)
install(
  TARGETS path_generate_server
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(leader_pos_server
  src/leader_pos_server.cpp
)
ament_target_dependencies(leader_pos_server
  rclcpp
  rclcpp_action
  swerm_msgs
  nav_msgs
  geometry_msgs
)
install(
  TARGETS leader_pos_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models
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
