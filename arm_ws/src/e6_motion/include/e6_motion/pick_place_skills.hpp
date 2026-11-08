// 抓取 / 放置的"基本技能"：回 home、接近并直线下降、直线移动、attach / detach。
// 原来写在 pick_place.cpp 里（P2），C 项目的 task_executor 也要用，所以挪到头文件里，两边共用。
//
// 两种运动方式：
//   关节空间规划（plan）：只保证起点、终点，中间末端走的是弧线，适合"大范围移动"、绕障碍物
//   笛卡尔路径（computeCartesianPath）：末端严格走直线，适合"靠近 / 离开物体"这种短距离、不能乱晃的动作
//
// 关键难点：同一个"方块上方"的位姿，机械臂有很多种姿态（多组 IK 解）都能到达，
// 但只有一部分姿态能从那里"直线下降"——其他的会中途碰到关节极限，或者 IK 跳到另一组解（某关节突然转 360°）。
// 所以到达上方之前，先"试算"：挑一个能顺利直线下降的姿态，再规划过去（见 findApproachJoints）。
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

namespace e6_motion
{

// ---------- 场景参数（单位：米，坐标系：dummy_link） ----------
constexpr double kCubeSize = 0.04;
// 末端（Link6 法兰）相对方块中心的高度。还没有夹爪，假想夹爪长 6 cm：法兰停在方块顶面上方 6 cm
constexpr double kGraspHeight = kCubeSize / 2 + 0.06;
constexpr double kApproachHeight = kGraspHeight + 0.10;  // 接近点：再高 10 cm，从这里直线下降
// 直线移动 10 cm 时，正常每个关节只转 30° 左右；超过 90° 说明 IK 跳到了另一组解
constexpr double kMaxStraightTurn = M_PI / 2;
constexpr int kMaxCandidates = 30;  // 最多试算多少个候选姿态
constexpr int kMaxPlanAttempts = 3;  // 规划失败时最多重新规划几次

// 末端朝正下方的位姿：绕 x 轴转 180°，Link6 的 z 轴就从朝上变成朝下
inline geometry_msgs::msg::Pose downPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = 1.0;  // 四元数 (x, y, z, w) = (1, 0, 0, 0) 就是"绕 x 轴转 180°"
  p.orientation.w = 0.0;
  return p;
}

// 一条关节轨迹里，起点到终点转动最大的那个关节转了多少（弧度）
inline double maxJointTurn(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto & pts = trajectory.joint_trajectory.points;
  double max_turn = 0.0;
  for (size_t j = 0; j < pts.front().positions.size(); ++j) {
    max_turn = std::max(max_turn, std::abs(pts.back().positions[j] - pts.front().positions[j]));
  }
  return max_turn;
}

class PickPlace
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  PickPlace(const rclcpp::Node::SharedPtr & node, MoveGroupInterface & arm)
  : logger_(node->get_logger()), arm_(arm),
    ik_client_(node->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik")) {}

  bool moveHome(const std::string & step)
  {
    arm_.setNamedTarget("home");
    return planAndExecute(step);
  }

  // 先到 above（物体上方），再直线下降到 down
  bool approachAndDescend(
    const geometry_msgs::msg::Pose & above, const geometry_msgs::msg::Pose & down, const std::string & step)
  {
    const auto joints = findApproachJoints(above, down, step);
    if (!joints) {
      return false;
    }
    // 目标直接给关节角（已经试算过的那个姿态），规划器负责找一条不碰撞的路过去
    arm_.setJointValueTarget(*joints);
    return planAndExecute(step + "：到上方") && moveStraight(down, step + "：直线下降");
  }

  // 只试算、不动：能不能到达 above、并从那里直线下降到 down（用来挑选空地）
  bool canReach(const geometry_msgs::msg::Pose & above, const geometry_msgs::msg::Pose & down, const std::string & step)
  {
    return findApproachJoints(above, down, step).has_value();
  }

  // 笛卡尔路径：末端从当前位置沿直线走到 target，朝向保持不变
  bool moveStraight(const geometry_msgs::msg::Pose & target_in, const std::string & step)
  {
    geometry_msgs::msg::Pose target = target_in;
    target.orientation = arm_.getCurrentPose().pose.orientation;  // 只改位置，朝向保持当前的
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = computeStraight(target, trajectory);
    // 安全检查：试算时已经检查过，这里执行前再查一次
    if (fraction < 0.99 || maxJointTurn(trajectory) > kMaxStraightTurn) {
      RCLCPP_ERROR(logger_, "[%s] 直线路径异常（完成 %.0f%%），放弃执行", step.c_str(), fraction * 100);
      return false;
    }
    RCLCPP_INFO(logger_, "[%s] 直线路径 100%%，关节最大转动 %.1f°",
      step.c_str(), maxJointTurn(trajectory) * 180.0 / M_PI);
    return execute(trajectory, step);
  }

  bool attach(const std::string & object_id)
  {
    // 把方块"挂"到 Link6 上：之后规划时，方块跟着末端一起动，也会一起做碰撞检测
    const bool ok = arm_.attachObject(object_id, arm_.getEndEffectorLink());
    RCLCPP_INFO(logger_, ok ? "已抓住 %s" : "抓取 %s 失败", object_id.c_str());
    return ok;
  }

  bool detach(const std::string & object_id)
  {
    const bool ok = arm_.detachObject(object_id);
    RCLCPP_INFO(logger_, ok ? "已放下 %s" : "放下 %s 失败", object_id.c_str());
    return ok;
  }

private:
  // 试算：找一个"能到达 above、并且能从 above 直线下降到 down"的姿态（6 个关节角）
  //   1. 用不同的起点求 IK（第一个起点 = 当前姿态，之后随机），move_group 会顺便检查碰撞
  //   2. 假装机械臂已经在这个姿态，试算直线下降（只计算、不执行）
  //   3. 直线 100% 可行、且没有关节突然大转 → 就用它
  std::optional<std::vector<double>> findApproachJoints(
    const geometry_msgs::msg::Pose & above, const geometry_msgs::msg::Pose & down, const std::string & step)
  {
    const auto model = arm_.getRobotModel();
    const auto * group = model->getJointModelGroup(arm_.getName());
    moveit::core::RobotState seed(*arm_.getCurrentState());

    std::optional<std::vector<double>> found;
    for (int i = 0; i < kMaxCandidates && !found; ++i) {
      if (i > 0) {
        seed.setToRandomPositions(group, rng_);  // 换一个随机起点，IK 就可能给出另一组解
      }
      const auto candidate = solveIk(above, seed);
      if (!candidate) {
        continue;
      }
      moveit::core::RobotState start(model);
      start.setToDefaultValues();
      start.setJointGroupPositions(group, *candidate);
      arm_.setStartState(start);  // 让 computeCartesianPath 从这个候选姿态开始算
      moveit_msgs::msg::RobotTrajectory trajectory;
      geometry_msgs::msg::Pose target = down;
      target.orientation = above.orientation;
      const double fraction = computeStraight(target, trajectory);
      if (fraction >= 0.99 && maxJointTurn(trajectory) <= kMaxStraightTurn) {
        RCLCPP_INFO(logger_, "[%s] 试算到第 %d 个候选姿态，可以直线下降", step.c_str(), i + 1);
        found = candidate;
      }
    }
    arm_.setStartStateToCurrentState();  // 试算完，起点恢复成"当前实际姿态"
    if (!found) {
      RCLCPP_ERROR(logger_, "[%s] 试算了 %d 个姿态，都不能直线下降", step.c_str(), kMaxCandidates);
    }
    return found;
  }

  // 调 move_group 的 /compute_ik 服务：给定末端位姿 + IK 起点，返回一组不碰撞的关节角
  std::optional<std::vector<double>> solveIk(
    const geometry_msgs::msg::Pose & pose, const moveit::core::RobotState & seed)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name = arm_.getName();
    request->ik_request.avoid_collisions = true;
    request->ik_request.timeout.nanosec = 50'000'000;  // 50 ms
    request->ik_request.pose_stamped.header.frame_id = arm_.getPlanningFrame();
    request->ik_request.pose_stamped.pose = pose;
    moveit::core::robotStateToRobotStateMsg(seed, request->ik_request.robot_state);

    // 服务的回复由另一个线程（spinner）处理，这里只需要等 future
    auto future = ik_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      return std::nullopt;
    }
    const auto response = future.get();
    if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      return std::nullopt;
    }
    moveit::core::RobotState solution(arm_.getRobotModel());
    moveit::core::robotStateMsgToRobotState(response->solution, solution);
    std::vector<double> joints;
    solution.copyJointGroupPositions(arm_.getName(), joints);
    return joints;
  }

  double computeStraight(const geometry_msgs::msg::Pose & target, moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    // eef_step = 0.005：每 5 mm 取一个点求 IK；jump_threshold = 0：不检查（我们自己用 maxJointTurn 检查）
    // 返回值 fraction = 直线上"能求出 IK 且不碰撞"的部分占多少，1.0 表示整条直线都可行
    return arm_.computeCartesianPath({target}, 0.005, 0.0, trajectory);
  }

  bool planAndExecute(const std::string & step)
  {
    // RRT 是随机的：偶尔找到的路径没通过最后的碰撞验证（比如拿着方块贴着箱子过去），再规划一次通常就好了
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= kMaxPlanAttempts && !planned; ++attempt) {
      planned = arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if (!planned) {
        RCLCPP_WARN(logger_, "[%s] 第 %d 次规划失败%s", step.c_str(), attempt,
          attempt < kMaxPlanAttempts ? "，重新规划" : "");
      }
    }
    if (!planned) {
      RCLCPP_ERROR(logger_, "[%s] 规划失败", step.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "[%s] 规划成功，%zu 个点", step.c_str(),
      plan.trajectory_.joint_trajectory.points.size());
    return execute(plan.trajectory_, step);
  }

  bool execute(const moveit_msgs::msg::RobotTrajectory & trajectory, const std::string & step)
  {
    if (arm_.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "[%s] 执行失败", step.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "[%s] 完成", step.c_str());
    return true;
  }

  rclcpp::Logger logger_;
  MoveGroupInterface & arm_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  random_numbers::RandomNumberGenerator rng_;
};

}  // namespace e6_motion
