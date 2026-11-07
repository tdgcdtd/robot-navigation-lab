import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    robot_sim_share = get_package_share_directory("robot_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # 启动参数 world：要加载的世界文件名（在 worlds/ 目录下），默认是原来的单障碍物场景。
    # 用法：ros2 launch robot_sim sim.launch.py world:=nav_world.sdf
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="robot_world.sdf",
        description="World file name under robot_sim/worlds"
    )
    world_file = PathJoinSubstitution(
        [robot_sim_share, "worlds", LaunchConfiguration("world")]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world_file]  # -r: 启动后立即开始仿真
        }.items()
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # 小车的坐标变换（my_robot/odom -> my_robot/chassis），RViz 显示小车位置要用
            "/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        # ROS 里的坐标变换统一发布在 /tf 话题上
        remappings=[("/model/my_robot/tf", "/tf")],
        output="screen"
    )

    return LaunchDescription([
        world_arg,
        gazebo,
        bridge
    ])