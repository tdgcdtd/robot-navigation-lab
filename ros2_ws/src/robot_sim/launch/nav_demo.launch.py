import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


# 一键启动完整演示：Gazebo 仿真 + A* 规划节点 + 路径跟踪节点 + RViz
def generate_launch_description():

    robot_sim_share = get_package_share_directory("robot_sim")

    # 复用已有的 sim.launch.py（Gazebo + ros_gz_bridge）
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_sim_share, "launch", "sim.launch.py")
        ),
        launch_arguments={"world": "nav_world.sdf"}.items()
    )

    planner = Node(
        package="path_planner",
        executable="path_publisher",
        # 地图、障碍物、默认目标都在这个 YAML 文件里，和 nav_world.sdf 对应
        parameters=[os.path.join(robot_sim_share, "config", "nav_world.yaml")],
        output="screen"
    )

    # world -> my_robot/odom 的静态坐标变换。
    # 小车出生在 world 原点、朝向 0，所以两个坐标系完全重合（平移 0、旋转 0）。
    world_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "world", "--child-frame-id", "my_robot/odom"],
        output="screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(robot_sim_share, "rviz", "nav.rviz")],
        # WSLg 下 Qt 默认用 Wayland 开窗口，而 RViz 的 3D 引擎 (OGRE) 用 X11 画图，
        # 两者不匹配会导致 RViz 启动即崩溃。强制 Qt 也走 X11 (xcb) 就能正常显示。
        additional_env={"QT_QPA_PLATFORM": "xcb"},
        output="screen"
    )

    follower = Node(
        package="robot_control",
        executable="path_follower",
        output="screen"
    )

    return LaunchDescription([
        sim,
        world_to_odom,
        planner,
        follower,
        rviz
    ])
