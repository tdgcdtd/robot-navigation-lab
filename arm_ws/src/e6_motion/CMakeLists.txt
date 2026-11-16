cmake_minimum_required(VERSION 3.8)
project(e6_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(move_demo src/move_demo.cpp)
target_compile_features(move_demo PUBLIC cxx_std_17)
ament_target_dependencies(move_demo rclcpp geometry_msgs moveit_ros_planning_interface)

add_executable(scene_demo src/scene_demo.cpp)
target_compile_features(scene_demo PUBLIC cxx_std_17)
ament_target_dependencies(scene_demo rclcpp geometry_msgs moveit_msgs shape_msgs moveit_ros_planning_interface)

add_executable(pick_place src/pick_place.cpp)
target_compile_features(pick_place PUBLIC cxx_std_17)
ament_target_dependencies(pick_place rclcpp geometry_msgs moveit_msgs shape_msgs std_srvs moveit_ros_planning_interface)

# C 项目：接收 /task_command 指令执行抓放
add_executable(task_executor src/task_executor.cpp)
target_compile_features(task_executor PUBLIC cxx_std_17)
ament_target_dependencies(task_executor rclcpp geometry_msgs moveit_msgs shape_msgs std_msgs std_srvs moveit_ros_planning_interface)

install(TARGETS move_demo scene_demo pick_place task_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
