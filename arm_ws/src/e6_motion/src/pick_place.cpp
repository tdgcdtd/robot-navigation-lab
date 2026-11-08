// P2 M5 第三步：抓取 - 放置动作序列（暂时没有夹爪，用 attach / detach 模拟"抓住 / 松开"）
//
// 场景：桌子 + 箱子 + 两个带编号的 4 cm 小方块放在箱子的一侧
// 动作：home → 对每个方块：按编号查放置位置 → 上方 → 直线下降 → 抓 → 抬起 → 翻过箱子到放置点 → 下降 → 放 → 抬起 → home
//
// 基本技能（PickPlace 类、downPose 等）在 include/e6_motion/pick_place_skills.hpp，
// 和 C 项目的 task_executor 共用。

#include <algorithm>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

#include "e6_motion/pick_place_skills.hpp"
#include "e6_motion/scene_utils.hpp"

using e6_motion::downPose;
using e6_motion::kApproachHeight;
using e6_motion::kCubeSize;
using e6_motion::kGraspHeight;
using e6_motion::makeBox;
using e6_motion::PickPlace;
using moveit::planning_interface::MoveGroupInterface;

namespace
{

  struct Point2
  {
    double x;
    double y;
  };
  
  struct Cube
  {
    std::string name;
    int number;
    Point2 pick;
  };
}  // namespace

int main(int argc, char ** argv)
{
  // 桌上有什么：每个方块的名字、编号、抓取位置
  const std::vector<Cube> cubes = {
    {"cube_1", 1, {0.25, 0.15}},
    {"cube_2", 2, {0.30, 0.20}},
  };

  // 分拣规则：编号 → 放置位置（想换去处，只改这里）
  const std::map<int, Point2> to_where = {
    {1, {0.25, -0.15}},
    {2, {0.30, -0.20}},
  };

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  // plan / execute / 服务调用都会阻塞等待，回调交给另一个线程处理
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  MoveGroupInterface arm(node, "me6_group");
  const double velocity_scaling = node->get_parameter_or("velocity_scaling", 0.2);
  const double scaling = std::clamp(velocity_scaling, 0.05, 1.0);
  arm.setMaxVelocityScalingFactor(scaling);
  arm.setMaxAccelerationScalingFactor(scaling);
  RCLCPP_INFO(logger, "当前速度：%.2f ,请求速度：%.2f", scaling, velocity_scaling);

  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(5);
  arm.startStateMonitor();  // 一开始就订阅 /joint_states，保证随时读得到当前姿态

  // 场景：桌子 + 箱子 + 每个方块
  const std::string frame = arm.getPlanningFrame();  // makeBox 要用到它，所以场景要在这之后再搭
  for (const Cube & cube : cubes) {
    arm.detachObject(cube.name);  // 如果上一次运行中途失败、方块还挂在手上，先放下（没挂着时会打印一条无害的报错）
  }
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(makeBox("table", frame, 0.0, 0.0, -0.011, 0.8, 0.8, 0.02));
  // 箱子：宽 10 cm、厚 8 cm、高 30 cm。不能太薄：规划器沿路径每隔一段才检查一次碰撞，
  // 太薄的墙可能"漏"在两个检查点之间，最后整条路径验证时才发现碰撞 → 规划失败
  objects.push_back(makeBox("box", frame, 0.25, 0.0, 0.15, 0.10, 0.08, 0.30));
  for (const Cube & cube : cubes) {
    objects.push_back(makeBox(cube.name, frame, cube.pick.x, cube.pick.y, kCubeSize / 2,
      kCubeSize, kCubeSize, kCubeSize));
  }
  const bool scene_ok = e6_motion::applyCollisionObjects(node, objects);
  if (!scene_ok) {
    RCLCPP_ERROR(logger, "加入场景失败，退出");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  RCLCPP_INFO(logger, "场景已就绪：桌子、箱子、%zu 个方块", cubes.size());

  PickPlace task(node, arm);
  bool ok = task.moveHome("回 home");

  for (const Cube & cube : cubes) {
    if (!ok) {
      break;  // 前面已经失败了，后面的方块都不做
    }

    // ① 按编号查放置位置
    const auto it = to_where.find(cube.number);
    if (it == to_where.end()) {
      RCLCPP_ERROR(logger, "分拣规则里没有编号 %d（%s），停止", cube.number, cube.name.c_str());
      ok = false;
      break;
    }
    const Point2 place = it->second;  // 放在 if 外面，后面的步骤才用得到
    RCLCPP_INFO(logger, "[%s] 编号 %d → 放到 (%.2f, %.2f)", cube.name.c_str(), cube.number, place.x, place.y);

    // ② 抓：接近方块 → 直线下降 → attach → 直线抬起
    ok = task.approachAndDescend(downPose(cube.pick.x, cube.pick.y, kApproachHeight),
        downPose(cube.pick.x, cube.pick.y, kGraspHeight), cube.name + " 接近") &&
      task.attach(cube.name) &&
      task.moveStraight(downPose(cube.pick.x, cube.pick.y, kApproachHeight), cube.name + " 抬起");

    // ③ 放：翻过箱子到放置点 → 直线下降 → detach → 直线抬起
    ok = ok &&
      task.approachAndDescend(downPose(place.x, place.y, kApproachHeight),
        downPose(place.x, place.y, kGraspHeight), cube.name + " 翻过箱子到放置点") &&
      task.detach(cube.name) &&
      task.moveStraight(downPose(place.x, place.y, kApproachHeight), cube.name + " 放下后抬起");
    // && 的"短路"：前一步失败（false），后面的步骤都不会执行 —— 和 P1 A* 里讲的 || 短路是同一个规则
  }

  ok = ok && task.moveHome("回 home");

  RCLCPP_INFO(logger, ok ? "分拣全部完成" : "中途失败，已停止");

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
