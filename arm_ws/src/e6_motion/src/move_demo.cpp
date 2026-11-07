// P2 M5：用代码控制 E6（代替在 RViz 里拖小球、点 Plan / Execute）
//
// 第一步做两件事：
//   ① 移动到预设姿态 "home"（SRDF 里定义的一组关节角）
//   ② 移动到指定的末端位置（给出 Link6 在 dummy_link 坐标系下的位置 + 朝向，MoveIt 负责求 IK 和规划）

#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

using moveit::planning_interface::MoveGroupInterface;

namespace
{

// 规划 + 执行，任何一步失败都返回 false。
// 对应 RViz 里的两个按钮：Plan → Execute。
bool planAndExecute(MoveGroupInterface & arm, const rclcpp::Logger & logger, const std::string & step)
{
  MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] 规划失败", step.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "[%s] 规划成功：轨迹 %zu 个点，用时 %.3f s",
    step.c_str(), plan.trajectory_.joint_trajectory.points.size(), plan.planning_time_);

  // execute() 内部就是发一个 FollowJointTrajectory action，等它返回结果
  if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "[%s] 执行失败", step.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "[%s] 执行完成", step.c_str());
  return true;
}

void logCurrentPose(MoveGroupInterface & arm, const rclcpp::Logger & logger)
{
  const auto p = arm.getCurrentPose().pose.position;
  RCLCPP_INFO(logger, "当前末端位置（%s 坐标系）：x=%.3f y=%.3f z=%.3f",
    arm.getPlanningFrame().c_str(), p.x, p.y, p.z);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "move_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  // 为什么要单独开一个线程 spin？
  // arm.plan() / arm.execute() 会一直等到结果回来才返回（阻塞）。
  // 等待期间，关节状态、action 结果这些消息还得有人处理（调用回调）。
  // 如果只在主线程里 spin，主线程正卡在 execute() 里，回调永远不会被执行 → 永远等不到结果。
  // （和 P1 错题"单线程执行器：一个回调卡住，其他回调都得等"是同一个道理）
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  // "me6_group" 是 SRDF 里定义的规划组：从 dummy_link 到 Link6 的 6 个关节
  MoveGroupInterface arm(node, "me6_group");
  arm.setMaxVelocityScalingFactor(0.2);      // 速度只用最大值的 20%（RViz 里的 Velocity Scaling）
  arm.setMaxAccelerationScalingFactor(0.2);
  arm.setPlanningTime(5.0);

  RCLCPP_INFO(logger, "规划坐标系：%s，末端：%s",
    arm.getPlanningFrame().c_str(), arm.getEndEffectorLink().c_str());

  bool ok = true;

  // ① 预设姿态：直接给 6 个关节角（关节空间目标），不需要 IK
  arm.setNamedTarget("home");
  ok = planAndExecute(arm, logger, "1 回到 home");
  if (ok) {
    logCurrentPose(arm, logger);
  }

  // ② 末端位置目标：只说"手要到哪、朝哪"，6 个关节角由 IK 求解器去算
  if (ok) {
    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;  // 朝向保持和 home 一样
    target.position.x = 0.25;
    target.position.y = 0.15;
    target.position.z = 0.25;
    arm.setPoseTarget(target);
    ok = planAndExecute(arm, logger, "2 末端到 (0.25, 0.15, 0.25)");
    if (ok) {
      logCurrentPose(arm, logger);
    }
  }

  RCLCPP_INFO(logger, ok ? "全部完成" : "中途失败，已停止");

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
