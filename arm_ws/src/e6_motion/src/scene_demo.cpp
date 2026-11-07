// P2 M5 第二步：在规划场景里放桌子和箱子，让 MoveIt 自动绕开
//
// 流程：
//   ① 往规划场景里加两个碰撞体：桌面（机械臂底座下面）+ 一堵"箱子墙"（挡在 A、B 两点之间）
//   ② 回 home → 到 A 点 → 到 B 点（A、B 在墙的两侧，直线过去会撞墙）
//   ③ 打印每段轨迹中末端到过的最高点，证明它是"翻过去"的
//
// 注意：规划场景是 MoveIt 脑子里的"地图"，Gazebo 里不会出现这些物体（就像 P1 的 YAML 和 SDF）。

#include <algorithm>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "e6_motion/scene_utils.hpp"

using e6_motion::makeBox;
using moveit::planning_interface::MoveGroupInterface;

namespace
{

// 对轨迹上的每个点做一次正运动学（FK），找出末端（Link6）到过的最高 z
double maxEndEffectorZ(MoveGroupInterface & arm, const MoveGroupInterface::Plan & plan)
{
  moveit::core::RobotState state(arm.getRobotModel());
  const auto * group = state.getJointModelGroup(arm.getName());
  double max_z = -1e9;
  for (const auto & point : plan.trajectory_.joint_trajectory.points) {
    state.setJointGroupPositions(group, point.positions);
    state.update();  // 根据关节角重新计算每个 link 的位置（就是 FK）
    max_z = std::max(max_z, state.getGlobalLinkTransform(arm.getEndEffectorLink()).translation().z());
  }
  return max_z;
}

bool planAndExecute(MoveGroupInterface & arm, const rclcpp::Logger & logger, const std::string & step)
{
  MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] 规划失败", step.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "[%s] 规划成功：轨迹 %zu 个点，末端最高到 z=%.3f m",
    step.c_str(), plan.trajectory_.joint_trajectory.points.size(), maxEndEffectorZ(arm, plan));

  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] 执行失败", step.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "[%s] 执行完成", step.c_str());
  return true;
}

bool moveToPosition(
  MoveGroupInterface & arm, const rclcpp::Logger & logger, const std::string & step,
  const geometry_msgs::msg::Quaternion & orientation, double x, double y, double z)
{
  geometry_msgs::msg::Pose target;
  target.orientation = orientation;
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;
  arm.setPoseTarget(target);
  return planAndExecute(arm, logger, step);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "scene_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  // 和 move_demo 一样：plan / execute 会阻塞，回调交给另一个线程处理
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  MoveGroupInterface arm(node, "me6_group");
  arm.setMaxVelocityScalingFactor(0.2);
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setPlanningTime(10.0);        // 有障碍物时 RRT 可能要多找一会儿
  arm.setNumPlanningAttempts(5);    // 规划 5 次取最好的一条

  // ① 往规划场景里加碰撞体
  const std::string frame = arm.getPlanningFrame();
  const bool scene_ok = e6_motion::applyCollisionObjects(node, {
    // 桌面：80×80 cm、厚 2 cm，顶面在 z = -0.001（比底座低 1 mm，避免"一开始就算碰撞"）
    makeBox("table", frame, 0.0, 0.0, -0.011, 0.8, 0.8, 0.02),
    // 箱子墙：在 A、B 两点正中间，宽 10 cm、厚 8 cm、高 32 cm（太薄的话规划器的碰撞检查可能"漏"掉它）
    makeBox("box", frame, 0.25, 0.0, 0.16, 0.10, 0.08, 0.32),
  });
  if (!scene_ok) {
    RCLCPP_ERROR(logger, "加入场景失败，退出");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  RCLCPP_INFO(logger, "已加入桌子和箱子（箱子顶面 z=0.32 m）");

  bool ok = true;

  arm.setNamedTarget("home");
  ok = planAndExecute(arm, logger, "1 回到 home");

  // 朝向保持 home 时的朝向，只改位置
  const auto orientation = arm.getCurrentPose().pose.orientation;

  if (ok) {
    ok = moveToPosition(arm, logger, "2 到 A 点 (0.25, 0.15, 0.20)", orientation, 0.25, 0.15, 0.20);
  }
  if (ok) {
    // A → B：直线会穿过箱子，MoveIt 必须绕开（从上面翻过去，或者绕到旁边）
    ok = moveToPosition(arm, logger, "3 到 B 点 (0.25, -0.15, 0.20)", orientation, 0.25, -0.15, 0.20);
  }

  RCLCPP_INFO(logger, ok ? "全部完成" : "中途失败，已停止");

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
