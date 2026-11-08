"""C 项目第 1 步：一句话指挥机械臂（文字版）。

启动 planner_node（一句话 → 指令）+ task_executor（指令 → MoveIt 抓放），两者读同一份 world.yaml。
前提：move_group 已经在运行（mock：moveit_demo.launch.py；Gazebo：moveit_gazebo.launch.py）。
打字的 text_input 要在另一个终端单独运行：ros2 run e6_llm text_input

用法：
  ros2 launch e6_llm command.launch.py use_sim_time:=false           # mock（RViz）
  ros2 launch e6_llm command.launch.py                               # Gazebo（默认 use_sim_time:=true）
  可选：backend:=rule（不用大模型）  velocity_scaling:=0.2
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    world = os.path.join(get_package_share_directory("e6_llm"), "config", "world.yaml")
    moveit_config = MoveItConfigsBuilder("me6_robot", package_name="me6_moveit").to_moveit_configs()
    use_sim_time = LaunchConfiguration("use_sim_time")

    planner = Node(
        package="e6_llm",
        executable="planner_node",
        output="screen",
        parameters=[world, {"backend": LaunchConfiguration("backend")}],
    )
    executor = Node(
        package="e6_motion",
        executable="task_executor",
        output="screen",
        parameters=[
            world,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {"velocity_scaling": ParameterValue(LaunchConfiguration("velocity_scaling"), value_type=float)},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("backend", default_value="auto"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.2"),
        planner,
        executor,
    ])
