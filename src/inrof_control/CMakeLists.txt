cmake_minimum_required(VERSION 3.8)
project(inrof_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(swerm_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_library(swerm_core
  src/swerm/boid_core.cpp
  src/swerm/follower_core.cpp
)
target_link_libraries(swerm_core
  Eigen3::Eigen
)

add_library(mppi_swerm
  src/mppi_swerm.cpp
)
target_link_libraries(mppi_swerm
  Eigen3::Eigen
  yaml-cpp
  swerm_core
)
add_executable(debug_node
  src/test_mppi.cpp
)
target_link_libraries(debug_node
  mppi_swerm
)
install(
  TARGETS debug_node
  DESTINATION lib/${PROJECT_NAME}
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

add_executable(leader_pos_server_mppi
  src/leader_pos_server_mppi.cpp
)
ament_target_dependencies(leader_pos_server_mppi
  rclcpp
  rclcpp_action
  swerm_msgs
  nav_msgs
  geometry_msgs
)
target_link_libraries(leader_pos_server_mppi
  Eigen3::Eigen
  mppi_swerm
)
install(
  TARGETS leader_pos_server_mppi
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})
install(PROGRAMS scripts/leader_rl_server.py DESTINATION lib/${PROJECT_NAME})

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
