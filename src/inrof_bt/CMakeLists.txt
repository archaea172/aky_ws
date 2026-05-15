cmake_minimum_required(VERSION 3.8)
project(inrof_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(swerm_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

# lifecycle server
add_executable(lifecycle_server
  src/lifecycle_server.cpp
)
ament_target_dependencies(lifecycle_server
  rclcpp
  rclcpp_action
  swerm_msgs
  lifecycle_msgs
)
install(TARGETS lifecycle_server DESTINATION lib/${PROJECT_NAME})

# bt library
add_library(bt_library
  src/bt/lifecycle_bt.cpp
  src/bt/leader_pos_bt.cpp
)
ament_target_dependencies(bt_library
  lifecycle_msgs
  swerm_msgs
)
target_link_libraries(bt_library behaviortree_ros2::behaviortree_ros2)

# bt node
add_executable(inrof_bt_node
  src/inrof_bt_node.cpp
)
ament_target_dependencies(inrof_bt_node
  rclcpp
  rclcpp_action
  lifecycle_msgs
  swerm_msgs
  ament_index_cpp
)
target_link_libraries(inrof_bt_node
  behaviortree_ros2::behaviortree_ros2
  bt_library
)
install(TARGETS inrof_bt_node DESTINATION lib/${PROJECT_NAME})

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
