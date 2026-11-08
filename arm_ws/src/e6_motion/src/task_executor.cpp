// C 项目：任务执行节点 —— 收到一条条简单指令，用 P2 的抓放技能执行
//
// 分工（每个节点只做一件事）：
//   planner_node（e6_llm 包，Python）：一句话 → 大模型 → 校验 → 拆成一条条指令，发到 /task_command
//   task_executor（这个文件）       ：/task_command → 查物体 / 目标位置 → MoveIt 抓放 → 把进度发到 /task_status
//
// 指令格式（一条消息 = 一步，纯文本，用空格分开）：
//   "pick_place red_cube left_zone"   把 red_cube 放到 left_zone
//   "home"                            回 home
//   "pick_place red_cube aside"       把 red_cube 挪到旁边的空地（具体放哪由这里找，见 findFreeSpot）
//
// 分工：大模型决定"做什么"（比如交换 = 先挪开一个），这里决定"放在哪"（几何问题交给代码，不让大模型报坐标）。
// 每次物体位置变了，就把"谁在哪"发到 /world_state，planner 写进提示词 —— 大模型只知道你告诉它的东西。
//
// 为什么不直接收大模型的原话？大模型偶尔会说错。执行端只认这两种固定格式、只认配置文件里有的名字，
// 其他一律拒绝 —— 大模型再怎么"胡说"，也让机械臂做不出清单以外的动作。
//
// 线程：订阅回调只把指令放进队列（很快就返回）；主线程从队列取指令、调用 MoveIt。
// 不能在回调里直接 plan / execute：它们要等回复，而回复正要靠这个回调线程来处理 → 死锁（和 P2 单独线程 spin 是同一个道理）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "e6_motion/pick_place_skills.hpp"
#include "e6_motion/scene_utils.hpp"

using e6_motion::downPose;
using e6_motion::kApproachHeight;
using e6_motion::kCubeSize;
using e6_motion::kGraspHeight;
using e6_motion::PickPlace;
using moveit::planning_interface::MoveGroupInterface;

namespace
{

const std::string kAside = "aside";  // 特殊目标：旁边的空地
// 空地离其他物体、离放置区域中心都至少这么远（两个方块宽），免得挤在一起，也不挡住区域
constexpr double kFreeClearance = 2 * kCubeSize;
// 找空地的范围（机械臂前方一块够得着的长方形，单位米）：x 0.20–0.34，y −0.24–0.24，每 4 cm 一个候选点
constexpr double kAsideMinX = 0.20, kAsideMaxX = 0.34, kAsideMaxY = 0.24, kAsideStep = 0.04;

// 桌上的一个位置：物体现在在哪，或者一个放置区域在哪（坐标系 dummy_link，单位米）
struct Place
{
  std::string label;  // 中文名，只用于打印
  double x = 0.0;
  double y = 0.0;
  std::vector<double> color;  // RGB，0–1，只有物体用（RViz 里的颜色）
};

struct Command
{
  std::string action;  // "pick_place" 或 "home"
  std::string object;
  std::string target;
};

// 解析一条指令文本。格式不对（多了 / 少了单词、未知动作）→ 返回空
std::optional<Command> parseCommand(const std::string & text)
{
  std::istringstream in(text);
  Command cmd;
  in >> cmd.action;
  if (cmd.action == "pick_place") {
    in >> cmd.object >> cmd.target;
    if (cmd.target.empty()) {
      return std::nullopt;
    }
  } else if (cmd.action != "home") {
    return std::nullopt;
  }
  std::string extra;
  if (in >> extra) {
    return std::nullopt;  // 后面还有多余的词
  }
  return cmd;
}

// 从参数里读出一组位置。YAML 里写的是
//   objects:
//     red_cube: {label: "红色方块", xy: [0.30, 0.06], color: [1.0, 0.1, 0.1]}
// ROS 2 会把它"压平"成参数 objects.red_cube.label / objects.red_cube.xy / objects.red_cube.color
std::map<std::string, Place> readPlaces(const rclcpp::Node::SharedPtr & node, const std::string & prefix)
{
  std::map<std::string, Place> places;
  const auto listed = node->list_parameters({prefix}, 0);  // 0 = 列出所有层级
  for (const std::string & name : listed.names) {
    const std::string rest = name.substr(prefix.size() + 1);  // "red_cube.xy"
    const auto dot = rest.find('.');
    if (dot == std::string::npos) {
      continue;
    }
    const std::string id = rest.substr(0, dot);
    const std::string key = rest.substr(dot + 1);
    const auto param = node->get_parameter(name);
    if (key == "label") {
      places[id].label = param.as_string();
    } else if (key == "xy") {
      const auto xy = param.as_double_array();
      if (xy.size() == 2) {
        places[id].x = xy[0];
        places[id].y = xy[1];
      }
    } else if (key == "color") {
      places[id].color = param.as_double_array();
    }
  }
  return places;
}

class TaskExecutor
{
public:
  TaskExecutor(const rclcpp::Node::SharedPtr & node, MoveGroupInterface & arm)
  : node_(node), arm_(arm), task_(node, arm),
    objects_(readPlaces(node, "objects")), targets_(readPlaces(node, "targets"))
  {
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("/task_status", 10);
    // transient_local：最后一条消息会被"留着"，planner 晚启动也能马上收到当前状态（和 P1 的 /map 一样）
    world_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/world_state", rclcpp::QoS(1).transient_local());
    command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/task_command", 10, [this](const std_msgs::msg::String & msg) {onCommand(msg);});
  }

  bool setupScene()
  {
    if (objects_.empty() || targets_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "参数里没有 objects / targets（launch 时加载 world.yaml 了吗？）");
      return false;
    }
    const std::string frame = arm_.getPlanningFrame();
    std::vector<moveit_msgs::msg::CollisionObject> scene;
    std::vector<moveit_msgs::msg::ObjectColor> colors;
    scene.push_back(e6_motion::makeBox("table", frame, 0.0, 0.0, -0.011, 0.8, 0.8, 0.02));
    for (const auto & [id, obj] : objects_) {
      arm_.detachObject(id);  // 上次中途失败、方块还挂在手上的话，先放下
      scene.push_back(e6_motion::makeBox(id, frame, obj.x, obj.y, kCubeSize / 2,
        kCubeSize, kCubeSize, kCubeSize));
      if (obj.color.size() == 3) {
        moveit_msgs::msg::ObjectColor c;
        c.id = id;
        c.color.r = static_cast<float>(obj.color[0]);
        c.color.g = static_cast<float>(obj.color[1]);
        c.color.b = static_cast<float>(obj.color[2]);
        c.color.a = 1.0F;
        colors.push_back(c);
      }
    }
    if (!e6_motion::applyCollisionObjects(node_, scene, colors)) {
      RCLCPP_ERROR(node_->get_logger(), "加入场景失败");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "场景已就绪：桌子 + %zu 个物体，%zu 个放置区域",
      objects_.size(), targets_.size());
    return task_.moveHome("回 home");
  }

  // 主线程循环：等指令 → 执行 → 汇报
  void run()
  {
    publishWorldState();
    report("准备好了，等待指令");
    while (rclcpp::ok()) {
      std::optional<Command> cmd;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // 最多等 0.2 秒就醒一次，好检查 Ctrl+C（rclcpp::ok() 变成 false）
        cv_.wait_for(lock, std::chrono::milliseconds(200), [this] {return !queue_.empty();});
        if (queue_.empty()) {
          continue;
        }
        cmd = queue_.front();
        queue_.pop_front();
      }
      const bool ok = execute(*cmd);
      if (!ok) {
        // 一步失败，后面排队的步骤很可能也不对了（比如方块没放到位），全部取消
        std::lock_guard<std::mutex> lock(mutex_);
        if (!queue_.empty()) {
          report("后面还有 " + std::to_string(queue_.size()) + " 步，已全部取消");
          queue_.clear();
        }
      }
    }
  }

private:
  // 回调（spinner 线程）：只检查格式、放进队列，马上返回
  void onCommand(const std_msgs::msg::String & msg)
  {
    const auto cmd = parseCommand(msg.data);
    if (!cmd) {
      report("拒绝：看不懂的指令「" + msg.data + "」");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(*cmd);
    cv_.notify_one();
  }

  bool execute(const Command & cmd)
  {
    if (cmd.action == "home") {
      report("开始：回 home");
      const bool ok = task_.moveHome("回 home");
      report(ok ? "完成：回 home" : "失败：回 home");
      return ok;
    }
    return pickPlace(cmd.object, cmd.target);
  }

  bool pickPlace(const std::string & object_id, const std::string & target_id)
  {
    // 第二道检查：名字必须在配置里（planner 已经查过一次，这里再查，不完全相信上游）
    const auto obj_it = objects_.find(object_id);
    const auto tgt_it = targets_.find(target_id);
    if (obj_it == objects_.end() || (tgt_it == targets_.end() && target_id != kAside)) {
      report("拒绝：不认识的物体或目标（" + object_id + " → " + target_id + "）");
      return false;
    }
    Place & obj = obj_it->second;

    Place tgt;
    if (target_id == kAside) {
      // 先找好空地再去抓：要是抓起来才发现没地方放，方块就卡在手上了
      const auto spot = findFreeSpot(object_id);
      if (!spot) {
        report("失败：附近找不到能放" + obj.label + "的空地");
        return false;
      }
      tgt = *spot;
    } else {
      tgt = tgt_it->second;
      if (std::hypot(obj.x - tgt.x, obj.y - tgt.y) < kCubeSize) {
        report("完成：" + obj.label + "本来就在" + tgt.label);
        return true;
      }
      // 目标位置被别的物体占了：硬放会叠在一起（碰撞），先拒绝
      for (const auto & [other_id, other] : objects_) {
        if (other_id != object_id && std::hypot(other.x - tgt.x, other.y - tgt.y) < kCubeSize * 1.5) {
          report("拒绝：" + tgt.label + "已经有" + other.label + "了");
          return false;
        }
      }
    }
    const std::string what = "把" + obj.label + "放到" + tgt.label;

    report("开始：" + what);
    const bool ok =
      task_.approachAndDescend(downPose(obj.x, obj.y, kApproachHeight),
        downPose(obj.x, obj.y, kGraspHeight), object_id + " 接近") &&
      task_.attach(object_id) &&
      task_.moveStraight(downPose(obj.x, obj.y, kApproachHeight), object_id + " 抬起") &&
      task_.approachAndDescend(downPose(tgt.x, tgt.y, kApproachHeight),
        downPose(tgt.x, tgt.y, kGraspHeight), object_id + " 到放置点") &&
      task_.detach(object_id) &&
      task_.moveStraight(downPose(tgt.x, tgt.y, kApproachHeight), object_id + " 放下后抬起");

    if (ok) {
      obj.x = tgt.x;  // 记住物体的新位置：下次再说"把红色方块放回右边"，就从这里抓
      obj.y = tgt.y;
      publishWorldState();
    }
    report((ok ? "完成：" : "失败：") + what);
    return ok;
  }

  // 在机械臂前方的长方形里按网格找一块空地：
  //   1. 候选点按"离物体现在的位置由近到远"排序（近的走得快）
  //   2. 离其他物体、离每个放置区域中心都 ≥ 8 cm（不挤、不挡区域）
  //   3. 最后用 IK 试算：够得着、能直线下降，才算数（只算不动）
  std::optional<Place> findFreeSpot(const std::string & object_id)
  {
    const Place & obj = objects_.at(object_id);
    std::vector<Place> candidates;
    for (double x = kAsideMinX; x <= kAsideMaxX + 1e-9; x += kAsideStep) {
      for (double y = -kAsideMaxY; y <= kAsideMaxY + 1e-9; y += kAsideStep) {
        Place p;
        p.label = "旁边空地";
        p.x = x;
        p.y = y;
        candidates.push_back(p);
      }
    }
    std::sort(candidates.begin(), candidates.end(), [&obj](const Place & a, const Place & b) {
        return std::hypot(a.x - obj.x, a.y - obj.y) < std::hypot(b.x - obj.x, b.y - obj.y);
      });

    for (Place & p : candidates) {
      if (std::hypot(p.x - obj.x, p.y - obj.y) < kFreeClearance) {
        continue;  // 离原位置太近，等于没挪
      }
      bool free = true;
      for (const auto & [other_id, other] : objects_) {
        free = free && (other_id == object_id || std::hypot(p.x - other.x, p.y - other.y) >= kFreeClearance);
      }
      for (const auto & [zone_id, zone] : targets_) {
        free = free && std::hypot(p.x - zone.x, p.y - zone.y) >= kFreeClearance;
      }
      if (free && task_.canReach(downPose(p.x, p.y, kApproachHeight), downPose(p.x, p.y, kGraspHeight), "试算空地")) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "旁边空地 (%.2f, %.2f)", p.x, p.y);
        p.label = buf;
        return p;
      }
    }
    return std::nullopt;
  }

  // 每个物体现在在哪个区域："red_cube right_zone"，不在任何区域就是 "red_cube table"，每个物体一行
  void publishWorldState()
  {
    std::string text;
    for (const auto & [id, obj] : objects_) {
      std::string where = "table";
      for (const auto & [zone_id, zone] : targets_) {
        if (std::hypot(obj.x - zone.x, obj.y - zone.y) < kCubeSize) {
          where = zone_id;
        }
      }
      text += id + " " + where + "\n";
    }
    std_msgs::msg::String msg;
    msg.data = text;
    world_pub_->publish(msg);
  }

  void report(const std::string & text)
  {
    RCLCPP_INFO(node_->get_logger(), "%s", text.c_str());
    std_msgs::msg::String msg;
    msg.data = "[执行] " + text;
    status_pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroupInterface & arm_;
  PickPlace task_;
  std::map<std::string, Place> objects_;  // 物体 id → 现在的位置（只在主线程里读写）
  std::map<std::string, Place> targets_;  // 放置区域 id → 位置
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr world_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  std::mutex mutex_;  // 保护 queue_：回调线程往里放，主线程往外取
  std::condition_variable cv_;
  std::deque<Command> queue_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "task_executor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  MoveGroupInterface arm(node, "me6_group");
  const double scaling = std::clamp(node->get_parameter_or("velocity_scaling", 0.2), 0.05, 1.0);
  arm.setMaxVelocityScalingFactor(scaling);
  arm.setMaxAccelerationScalingFactor(scaling);
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(5);
  arm.startStateMonitor();
  RCLCPP_INFO(node->get_logger(), "速度比例 %.2f", scaling);

  TaskExecutor task_executor(node, arm);
  int code = 1;
  if (task_executor.setupScene()) {
    task_executor.run();
    code = 0;
  }

  rclcpp::shutdown();
  spinner.join();
  return code;
}
