#include "path_planner/grid_map.hpp"
#include "path_planner/astar_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

// 规划节点：
//   - 持续记录小车当前位置（里程计）
//   - 启动后先规划到默认目标；之后每收到一个 /goal_pose（RViz 里点的目标），
//     就从小车"当前位置"重新规划
//   - 目标无效（地图外 / 障碍物或膨胀区里）时发布一条空路径，让跟踪节点停车
//   - 检测到 Gazebo Reset（仿真时间倒退）时，从小车新位置重新规划到当前目标
//   - 路径以 1 Hz 重复发布到 /planned_path；栅格地图发布到 /map 供 RViz 显示
class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode()
        : Node("path_publisher"),
          // 地图尺寸也是参数（默认值 = 原来写死的 20 x 20、0.5 m、原点 (-5, -5)）
          grid_map_(declare_parameter("map.width", 20),
                    declare_parameter("map.height", 20),
                    declare_parameter("map.resolution", 0.5),
                    declare_parameter("map.origin_x", -5.0),
                    declare_parameter("map.origin_y", -5.0))
    {
        // 参数：可以在命令行或 launch 文件里修改，不用重新编译
        default_goal_x_ = declare_parameter("goal_x", 3.5);
        default_goal_y_ = declare_parameter("goal_y", 1.0);
        plan_on_start_ = declare_parameter("plan_on_start", true);
        // true：8 邻域（可以斜着走）；false：只走上下左右
        allow_diagonal_ = declare_parameter("allow_diagonal", true);
        // 小车车身 0.8 x 0.5 m，外接圆半径约 0.47 m，再加 0.1 m 安全余量
        double robot_radius = declare_parameter("robot_radius", 0.47);
        double safety_margin = declare_parameter("safety_margin", 0.1);

        loadObstacles();
        grid_map_.inflate(robot_radius + safety_margin);

        path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        // 地图只发布一次。transient_local（"保留最后一条消息"）让之后才启动的
        // RViz 也能收到这条地图，否则晚启动的订阅者会错过它。
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable());
        map_pub_->publish(makeMapMessage());

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_robot/odometry",
            10,
            std::bind(&PathPlannerNode::odomCallback, this, std::placeholders::_1));

        // RViz 工具栏的 "2D Goal Pose" 会把点击的位置发布到 /goal_pose
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(&PathPlannerNode::goalCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(1s, std::bind(&PathPlannerNode::publishPath, this));

        RCLCPP_INFO(get_logger(), "Ready. Use \"2D Goal Pose\" in RViz to set a goal.");
    }

private:
    // 从参数读取障碍物列表，写进栅格地图。参数格式（见 robot_sim/config/nav_world.yaml）：
    //   obstacle_names: ["obstacle_1", "wall_north"]
    //   obstacle_1.center: [2.0, 1.0]    # 中心 x, y
    //   obstacle_1.size:   [1.0, 1.0]    # 长（x 方向）, 宽（y 方向）
    // 没有提供参数文件时，默认只有 obstacle_1（对应原来的 robot_world.sdf）
    void loadObstacles()
    {
        auto names = declare_parameter<std::vector<std::string>>(
            "obstacle_names", std::vector<std::string>{"obstacle_1"});

        for (const auto& name : names)
        {
            auto center = declare_parameter<std::vector<double>>(
                name + ".center", std::vector<double>{2.0, 1.0});
            auto size = declare_parameter<std::vector<double>>(
                name + ".size", std::vector<double>{1.0, 1.0});

            if (center.size() != 2 || size.size() != 2)
            {
                RCLCPP_ERROR(get_logger(), "Obstacle \"%s\": center and size must each have 2 numbers, skipped.",
                             name.c_str());
                continue;
            }

            // 中心 + 尺寸 -> 左下角和右上角
            double half_x = size[0] / 2.0;
            double half_y = size[1] / 2.0;
            grid_map_.setOccupiedRect(center[0] - half_x, center[1] - half_y,
                                      center[0] + half_x, center[1] + half_y);
            RCLCPP_INFO(get_logger(), "Obstacle %-14s center (%5.2f, %5.2f) size %.2f x %.2f",
                        name.c_str(), center[0], center[1], size[0], size[1]);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 注意：这里直接把 odom 坐标当成 world 坐标使用。
        // 只有小车出生在 world 原点、朝向为 0 时才成立（本项目正是如此）。
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // 检测 Gazebo 的 Reset：仿真时间会回到 0，所以里程计的时间戳会突然变小。
        // 这时小车已经回到起点，旧路径作废，要从新位置重新规划到当前目标。
        rclcpp::Time stamp(msg->header.stamp);
        bool time_jumped_back = has_odom_ && stamp < last_odom_stamp_;
        last_odom_stamp_ = stamp;
        if (time_jumped_back)
        {
            RCLCPP_WARN(get_logger(), "Simulation reset detected (time jumped back).");
            if (has_goal_)
            {
                planTo(goal_x_, goal_y_);
            }
            return;
        }

        if (!has_odom_)
        {
            has_odom_ = true;
            if (plan_on_start_)
            {
                planTo(default_goal_x_, default_goal_y_);
            }
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg->header.frame_id != "world")
        {
            RCLCPP_WARN(get_logger(), "Goal frame is \"%s\", expected \"world\". Check RViz Fixed Frame.",
                        msg->header.frame_id.c_str());
        }
        if (!has_odom_)
        {
            RCLCPP_WARN(get_logger(), "No odometry yet, cannot plan.");
            return;
        }
        if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y))
        {
            RCLCPP_ERROR(get_logger(), "Goal contains NaN/inf, ignored.");
            return;
        }
        goal_orientation_ = msg->pose.orientation;   // 记住 RViz 里拖出的箭头方向
        planTo(msg->pose.position.x, msg->pose.position.y);
    }

    // 从小车当前位置规划到 (goal_x, goal_y)。成功返回 true。
    // 失败时发布空路径（取消当前任务），让跟踪节点停车。
    bool planTo(double goal_x, double goal_y)
    {
        // 记住当前目标：仿真重置后要重新规划到它
        goal_x_ = goal_x;
        goal_y_ = goal_y;
        has_goal_ = true;

        RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f), robot at (%.2f, %.2f)",
                    goal_x, goal_y, robot_x_, robot_y_);

        int start_row, start_col, goal_row, goal_col;
        if (!grid_map_.worldToGrid(robot_x_, robot_y_, start_row, start_col))
        {
            return rejectGoal("Robot is outside the map.");
        }
        if (!grid_map_.worldToGrid(goal_x, goal_y, goal_row, goal_col))
        {
            return rejectGoal("Goal is outside the map.");
        }
        if (grid_map_.isOccupied(goal_row, goal_col))
        {
            return rejectGoal("Goal is inside an obstacle or its inflated zone.");
        }
        if (grid_map_.getCost(start_row, start_col) == GridMap::kOccupied)
        {
            return rejectGoal("Robot is inside an obstacle cell.");
        }
        if (grid_map_.getCost(start_row, start_col) == GridMap::kInflated)
        {
            // 小车已经在膨胀区里（比如被推进去了）：仍然允许规划，让它自己开出来
            RCLCPP_WARN(get_logger(), "Robot is inside the inflated zone, planning anyway.");
        }

        auto t0 = std::chrono::steady_clock::now();
        auto grid_path = astarSearch(grid_map_, start_row, start_col, goal_row, goal_col,
                                     allow_diagonal_);
        double plan_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        if (grid_path.empty())
        {
            return rejectGoal("No path found!");
        }

        // 路径拉直：去掉能被直线跳过的中间点，得到少量、任意角度的路径点
        auto corners = smoothPath(grid_map_, grid_path);
        path_msg_ = makePathMessage(corners, goal_x, goal_y);
        publishPath();  // 立刻发布一次，不用等定时器

        RCLCPP_INFO(get_logger(), "Planned in %.2f ms: %zu cells, length %.2f m -> %zu waypoints",
                    plan_ms, grid_path.size(), pathLength(grid_path) * grid_map_.resolution(),
                    path_msg_.poses.size());
        for (const auto& pose : path_msg_.poses)
        {
            RCLCPP_INFO(get_logger(), "  waypoint (%.2f, %.2f)",
                        pose.pose.position.x, pose.pose.position.y);
        }
        return true;
    }

    bool rejectGoal(const char* reason)
    {
        RCLCPP_ERROR(get_logger(), "%s Robot will stop.", reason);
        path_msg_ = nav_msgs::msg::Path();
        path_msg_.header.frame_id = "world";
        publishPath();  // 空路径 = 取消当前任务
        return false;
    }

    nav_msgs::msg::OccupancyGrid makeMapMessage()
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "world";
        map_msg.header.stamp = now();
        map_msg.info.resolution = grid_map_.resolution();
        map_msg.info.width = grid_map_.width();
        map_msg.info.height = grid_map_.height();
        // 地图左下角（第 0 行第 0 列格子的角）在 world 中的位置
        map_msg.info.origin.position.x = grid_map_.originX();
        map_msg.info.origin.position.y = grid_map_.originY();
        map_msg.info.origin.orientation.w = 1.0;

        // OccupancyGrid 的数据按"行优先"存放：先第 0 行从左到右，再第 1 行……
        // 和 GridMap 内部的 index = row * width + col 完全一致
        for (int row = 0; row < grid_map_.height(); ++row)
        {
            for (int col = 0; col < grid_map_.width(); ++col)
            {
                map_msg.data.push_back(grid_map_.getCost(row, col));
            }
        }
        return map_msg;
    }

    nav_msgs::msg::Path makePathMessage(const std::vector<std::pair<int, int>>& cells,
                                        double goal_x, double goal_y) const
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "world";

        // 跳过第 0 个格子：那是小车所在的格子，小车已经在那里了。
        // 但如果起点和终点在同一个格子里（只有 1 个格子），就从 0 开始，保证至少有终点。
        size_t first = (cells.size() == 1) ? 0 : 1;
        for (size_t i = first; i < cells.size(); ++i)
        {
            double x;
            double y;
            grid_map_.gridToWorld(cells[i].first, cells[i].second, x, y);

            // 最后一个点用精确的目标坐标，而不是目标所在格子的中心
            if (i + 1 == cells.size())
            {
                x = goal_x;
                y = goal_y;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation.w = 1.0;
            if (i + 1 == cells.size())
            {
                pose.pose.orientation = goal_orientation_;
            }
            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

    void publishPath()
    {
        // 还没有规划过任何路径时不发布（frame_id 为空说明还是初始状态）
        if (path_msg_.header.frame_id.empty())
        {
            return;
        }
        auto stamp = now();
        path_msg_.header.stamp = stamp;
        for (auto& pose : path_msg_.poses)
        {
            pose.header.stamp = stamp;
        }
        path_pub_->publish(path_msg_);
    }

    GridMap grid_map_;
    double default_goal_x_;
    double default_goal_y_;
    bool plan_on_start_;
    bool allow_diagonal_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool has_odom_ = false;
    rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
    nav_msgs::msg::Path path_msg_;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    geometry_msgs::msg::Quaternion goal_orientation_;   // 目标朝向（四元数）
    bool has_goal_ = false;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // 节点存在变量里，保证 rclcpp::shutdown() 时节点还活着
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
