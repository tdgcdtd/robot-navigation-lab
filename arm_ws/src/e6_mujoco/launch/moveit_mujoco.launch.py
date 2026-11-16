"""启动 MoveIt，让它把轨迹交给另一台电脑上的 MuJoCo 执行（代替假硬件 / Gazebo）。

用法：
  ros2 launch e6_mujoco moveit_mujoco.launch.py sim_host:=<MuJoCo 那台电脑的 Tailscale 地址>
  ros2 launch e6_mujoco moveit_mujoco.launch.py sim_host:=... rviz:=false      # 不开 RViz（远程启动时用）

和 me6_moveit 的 demo.launch.py 比，少了 ros2_control（假硬件 + 控制器），多了 bridge_node：
  robot_state_publisher  关节角 → TF（和原来一样）
  move_group             规划（和原来一样），轨迹发给 /me6_group_controller/follow_joint_trajectory
  bridge_node            接这个动作，转交给 MuJoCo；发布 /joint_states
之后照常启动 task_executor 等节点，记得加 use_sim_time:=false（时间用真实时钟，不是 Gazebo 的仿真时钟）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("me6_robot", package_name="me6_moveit").to_moveit_configs()

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[moveit_config.robot_description])

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                # MoveIt 默认"比预计时间多 20% + 0.5 s 还没执行完就取消"。
                # 我们的网络往返要 0.2~2.5 s，放宽到"3 倍 + 10 s"，否则会被误判成超时
                "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                "trajectory_execution.allowed_goal_duration_margin": 10.0,
            },
        ])

    bridge = Node(
        package="e6_mujoco", executable="bridge_node", output="screen",
        parameters=[{"sim_host": LaunchConfiguration("sim_host"), "sim_port": ParameterValue(LaunchConfiguration("sim_port"), value_type=int)}])

    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                    moveit_config.joint_limits],
        condition=IfCondition(LaunchConfiguration("rviz")))

    return LaunchDescription([
        DeclareLaunchArgument("sim_host", description="MuJoCo 服务所在电脑的地址"),
        DeclareLaunchArgument("sim_port", default_value="5600"),
        DeclareLaunchArgument("rviz", default_value="true"),
        rsp, move_group, bridge, rviz,
    ])
