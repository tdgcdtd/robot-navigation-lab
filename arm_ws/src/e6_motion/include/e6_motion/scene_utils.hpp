// 规划场景的小工具：造长方体碰撞体、把碰撞体加进 move_group 的规划场景
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace e6_motion
{

// 长方体碰撞体：中心 (x, y, z)，尺寸 (sx, sy, sz)，单位米，坐标系 = frame
inline moveit_msgs::msg::CollisionObject makeBox(
  const std::string & id, const std::string & frame,
  double x, double y, double z, double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;  // 不旋转

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

// 把碰撞体加进规划场景（调用 move_group 的 /apply_planning_scene 服务）。
//
// 为什么不用 MoveIt 自带的 PlanningSceneInterface::applyCollisionObjects()？
// 它内部用自己的方式等待回复，测试中偶尔一直等不到、程序卡住不动（表现为"运行后没有任何输出"）。
// 这里改用我们自己节点上的服务客户端：回复由 spinner 线程处理，我们只等 5 秒，超时就重试。
// colors（可选）：给物体上色，只影响 RViz 里的显示，不影响规划。
inline bool applyCollisionObjects(
  const rclcpp::Node::SharedPtr & node, const std::vector<moveit_msgs::msg::CollisionObject> & objects,
  const std::vector<moveit_msgs::msg::ObjectColor> & colors = {})
{
  auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "等不到 /apply_planning_scene 服务（move_group 启动了吗？）");
    return false;
  }

  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;  // 只是"增加"这些物体，不替换整个场景
  request->scene.world.collision_objects = objects;
  request->scene.object_colors = colors;

  for (int attempt = 1; attempt <= 3; ++attempt) {
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      return future.get()->success;
    }
    RCLCPP_WARN(node->get_logger(), "加入场景超时，第 %d 次重试", attempt);
  }
  return false;
}

}  // namespace e6_motion
