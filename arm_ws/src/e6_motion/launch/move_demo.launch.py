"""启动我们自己的运动控制节点。

用法：ros2 launch e6_motion move_demo.launch.py demo:=scene_demo / pick_place   （默认 demo:=move_demo）

前提：move_group 已经在运行（Gazebo 用 moveit_gazebo.launch.py，纯 RViz 用 moveit_demo.launch.py）。
MoveGroupInterface 需要知道机器人长什么样，所以要把 URDF / SRDF / 运动学参数传给这个节点。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 和 move_group 用同一份配置（越疆官方的 me6_moveit 包）
    moveit_config = MoveItConfigsBuilder("me6_robot", package_name="me6_moveit").to_moveit_configs()

    # Gazebo 里用仿真时钟（true）；moveit_demo（mock 硬件）用电脑时钟（false）
    use_sim_time = LaunchConfiguration("use_sim_time")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    demo = LaunchConfiguration("demo")  # 运行哪个程序：move_demo / scene_demo / pick_place

    move_demo = Node(
        package="e6_motion",
        executable=demo,
        output="screen",
        parameters=[
            moveit_config.robot_description,           # URDF：零件和关节
            moveit_config.robot_description_semantic,  # SRDF：规划组 me6_group、预设姿态 home
            moveit_config.robot_description_kinematics,  # IK 求解器（KDL）
            {"use_sim_time": use_sim_time},
            {"velocity_scaling": ParameterValue(velocity_scaling, value_type=float)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("demo", default_value="move_demo"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.2"),
        move_demo,
    ])
