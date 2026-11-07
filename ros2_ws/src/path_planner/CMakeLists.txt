cmake_minimum_required(VERSION 3.20)
project(path_planner)

# 默认用 Release（开启 -O2/-O3 优化）。不设置的话 colcon 默认不开优化，
# A* 会慢好几倍，性能测试结果也没有参考意义。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  # 单元测试（GoogleTest）。这里只跑我们自己写的测试，
  # 不启用 ament_lint_auto 的代码风格检查（cpplint/uncrustify 等），它们的格式要求和本项目不同。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_astar
    test/test_astar.cpp
    src/grid_map.cpp
    src/astar_planner.cpp
    src/astar_basic.cpp
  )
  target_include_directories(
    test_astar
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

add_executable(
  test_grid_map
  src/test_grid_map.cpp
  src/grid_map.cpp
  src/astar_planner.cpp
)

target_include_directories(
  test_grid_map 
  PRIVATE
  include
)

add_executable(
  path_publisher
  src/path_publisher.cpp
  src/grid_map.cpp
  src/astar_planner.cpp
)

target_include_directories(
  path_publisher 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  path_publisher 
  PRIVATE
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# A* 性能对比程序：第一版 vs 优化版（不依赖 ROS）
add_executable(
  benchmark_astar
  src/benchmark_astar.cpp
  src/grid_map.cpp
  src/astar_planner.cpp
  src/astar_basic.cpp
)

target_include_directories(
  benchmark_astar
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(
  TARGETS
    test_grid_map
    path_publisher
    benchmark_astar
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
