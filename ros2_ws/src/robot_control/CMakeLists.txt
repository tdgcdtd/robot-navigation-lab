cmake_minimum_required(VERSION 3.20)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(velocity_publisher src/velocity_publisher.cpp)

target_link_libraries(
  velocity_publisher
  PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

add_executable(cmd_vel_subscriber src/cmd_vel_subscriber.cpp)

target_link_libraries(
  cmd_vel_subscriber
  PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

add_executable(
  test_angle_utils
  src/test_angle_utils.cpp
  src/angle_utils.cpp
)

target_include_directories(
  test_angle_utils
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(
  go_to_goal
  src/go_to_goal.cpp
  src/angle_utils.cpp
  src/controller.cpp
)

target_include_directories(
  go_to_goal
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  go_to_goal
  PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_executable(
  path_follower
  src/path_follower.cpp
  src/angle_utils.cpp
  src/controller.cpp
)

target_include_directories(
  path_follower
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  path_follower
  PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
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

install(
  TARGETS 
    velocity_publisher
    cmd_vel_subscriber
    test_angle_utils
    go_to_goal
    path_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
