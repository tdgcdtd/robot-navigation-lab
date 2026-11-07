cmake_minimum_required(VERSION 3.16)

project(RobotNavigationLab)

add_executable(vector_test
    vector_test.cpp
    speed_utils.cpp
)
add_executable(robot_state_test
    robot_state_test.cpp
    Robot.cpp 
)
