#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_control/angle_utils.hpp"
#include "robot_control/controller.hpp"

using namespace std::chrono_literals;

// 路径跟踪节点：订阅 /planned_path，按顺序逐个开到每个路径点，到终点后停车。
// 收到新路径时切换过去；收到空路径（任务被取消）时停车；
// 检测到 Gazebo Reset 时丢弃旧路径，等待重新规划的路径。
class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower")
    {
        // 中间路径点：进入这个半径就切换到下一个点（不必精确到达）
        waypoint_tolerance_ = declare_parameter("waypoint_tolerance", 0.15);
        // 最终目标点：进入这个半径才算到达
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.1);
        // 朝向误差小于这个值（约 3°）就算转到位
        yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.05);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_robot/odometry",
            10,
            std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/planned_path",
            10,
            std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(100ms, std::bind(&PathFollower::controlLoop, this));

        // Ctrl+C 退出时先让小车停下
        get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() { publishStop(); });

        RCLCPP_INFO(get_logger(), "Waiting for /planned_path ...");
    }

private:
    struct Waypoint
    {
        double x;
        double y;
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 检测 Gazebo 的 Reset（仿真时间倒退）：小车被放回起点，正在跟踪的旧路径已经不对了。
        // 先停车并清空路径，等规划节点从新位置重新规划后发来新路径。
        rclcpp::Time stamp(msg->header.stamp);
        if (has_odom_ && stamp < last_odom_stamp_)
        {
            publishStop();
            waypoints_.clear();
            finished_ = false;
            rotating_ = false;
            RCLCPP_WARN(get_logger(), "Simulation reset detected, dropping old path and waiting for a new one.");
        }
        last_odom_stamp_ = stamp;

        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        yaw_ = quaternionToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        has_odom_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // 空路径 = 规划节点取消了任务（比如点的目标无效）：停车
        if (msg->poses.empty())
        {
            if (!waypoints_.empty())
            {
                publishStop();
                waypoints_.clear();
                finished_ = true;
                rotating_ = false;
                RCLCPP_WARN(get_logger(), "Path cancelled by planner, stopping.");
            }
            return;
        }

        std::vector<Waypoint> new_waypoints;
        for (const auto& pose : msg->poses)
        {
            new_waypoints.push_back({pose.pose.position.x, pose.pose.position.y});
        }

        // 最后一个路径点的朝向就是目标朝向（四元数 -> yaw）
        const auto& q = msg->poses.back().pose.orientation;
        double new_goal_yaw = quaternionToYaw(q.x, q.y, q.z, q.w);

        // 规划节点每秒都会重发同一条路径：内容相同就忽略，
        // 否则每次收到都从第一个点重新开始，小车会来回打转。
        if (isSamePath(new_waypoints, waypoints_) && new_goal_yaw == goal_yaw_)
        {
            return;
        }

        goal_yaw_ = new_goal_yaw;
        
        rotating_ = false;   // 新任务从"行驶"阶段开始

        // 新路径：替换旧路径，从第一个点重新开始
        waypoints_ = new_waypoints;
        current_index_ = 0;
        finished_ = false;
        RCLCPP_INFO(get_logger(), "Received new path with %zu waypoints", waypoints_.size());
    }

    static bool isSamePath(const std::vector<Waypoint>& a, const std::vector<Waypoint>& b)
    {
        if (a.size() != b.size())
        {
            return false;
        }
        for (size_t i = 0; i < a.size(); ++i)
        {
            // 重复发布的是同一条消息，坐标完全相同，可以直接用 == 比较
            if (a[i].x != b[i].x || a[i].y != b[i].y)
            {
                return false;
            }
        }
        return true;
    }

    void controlLoop()
    {
        // 正在退出时不再发速度，避免覆盖掉停车命令
        if (!rclcpp::ok())
        {
            return;
        }

        if (!has_odom_ || waypoints_.empty() || finished_)
        {
            return;
        }

        if (rotating_)
        {
            // 你来写：
            // 1. 算出朝向误差（目标 yaw − 当前 yaw），记得归一化
            double angle_error_temp = normalizeAngle(goal_yaw_ - yaw_);  
            // 2. 误差足够小（绝对值小于 yaw_tolerance_）→ 停车，finished_ = true，rotating_ = false，
            //    打印 "Goal reached"，return
            if(std::abs(angle_error_temp) < yaw_tolerance_)
            {
                publishStop();
                finished_ = true;
                rotating_ = false;

                RCLCPP_INFO(get_logger(), "Goal reached");
                return;
            }
            // 3. 否则 → 只转不走：算出角速度（提示：试试 computeVelocity(0.0, 误差, params_)），
            //    发布速度命令，return
            else
            {
                VelocityCommand angle_cmd = computeVelocity(0.0, angle_error_temp, params_);
                geometry_msgs::msg::Twist twist_temp;
                twist_temp.linear.x = angle_cmd.linear;
                twist_temp.angular.z = angle_cmd.angular;
                cmd_pub_->publish(twist_temp);

                return;
            }
        }

        const Waypoint& target = waypoints_[current_index_];
        const bool is_last = (current_index_ + 1 == waypoints_.size());

        double dx = target.x - x_;
        double dy = target.y - y_;
        double distance = std::hypot(dx, dy);

        // 到达判定
        if (is_last && distance < goal_tolerance_)
        {
            publishStop();
            rotating_ = true;
            RCLCPP_INFO(get_logger(), "Position reached, rotating to goal yaw");
            return;
        }
        if (!is_last && distance < waypoint_tolerance_)
        {
            ++current_index_;
            RCLCPP_INFO(get_logger(), "Waypoint %zu reached, heading to (%.2f, %.2f)",
                        current_index_, waypoints_[current_index_].x,
                        waypoints_[current_index_].y);
            return;  // 下一轮循环再朝新的目标点计算
        }

        double angle_error = normalizeAngle(std::atan2(dy, dx) - yaw_);
        VelocityCommand cmd = computeVelocity(distance, angle_error, params_);

        geometry_msgs::msg::Twist twist;
        twist.linear.x = cmd.linear;
        twist.angular.z = cmd.angular;
        cmd_pub_->publish(twist);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[%zu/%zu] pos=(%.2f, %.2f) yaw=%.1f deg | distance=%.2f angle_error=%.1f deg | v=%.2f w=%.2f",
            current_index_ + 1, waypoints_.size(), x_, y_, yaw_ * 180.0 / M_PI,
            distance, angle_error * 180.0 / M_PI, cmd.linear, cmd.angular);
    }

    void publishStop()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
    }

    // ===== 成员变量 =====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    bool has_odom_ = false;
    rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};

    std::vector<Waypoint> waypoints_;
    size_t current_index_ = 0;
    bool finished_ = false;

    double goal_yaw_ = 0.0;      // 目标朝向（从路径最后一个点算出来）
    bool rotating_ = false;      // true = 已经到达位置，正在原地转向
    double yaw_tolerance_;       // 朝向误差小于它，就算转到位了

    double waypoint_tolerance_;
    double goal_tolerance_;
    ControllerParams params_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // 节点存在变量里，保证 rclcpp::shutdown() 执行停车回调时节点还活着
    auto node = std::make_shared<PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
