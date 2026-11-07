#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_control/angle_utils.hpp"
#include "robot_control/controller.hpp"

using namespace std::chrono_literals;   // 这样就可以直接写 100ms

// 单目标点控制器：让小车开到 (goal_x, goal_y) 并停下
class GoToGoal : public rclcpp::Node
{
public:
    GoToGoal() : Node("go_to_goal")
    {
        goal_x_ = declare_parameter("goal_x", 1.0);
        goal_y_ = declare_parameter("goal_y", -1.0);
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.1);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_robot/odometry",
            10,
            std::bind(&GoToGoal::odomCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(100ms, std::bind(&GoToGoal::controlLoop, this));

        // 按 Ctrl+C 退出时先发一次零速度。
        // 否则 Gazebo 的差速驱动会一直保持最后一次收到的速度，小车停不下来。
        get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() { publishStop(); });
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        yaw_ = quaternionToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        has_odom_ = true;
    }

    void controlLoop()
    {
        // 正在退出时不再发速度，避免覆盖掉停车命令
        if (!rclcpp::ok())
        {
            return;
        }

        if (!has_odom_ || reached_) {
            return;
        }

        double dx = goal_x_ - x_;
        double dy = goal_y_ - y_;
        double distance = std::hypot(dx, dy);

        double target_angle = std::atan2(dy, dx);
        double angle_error = normalizeAngle(target_angle - yaw_);

        // 到达判定：距离小于容差就停车，之后不再控制
        if (distance < goal_tolerance_)
        {
            publishStop();
            reached_ = true;
            RCLCPP_INFO(get_logger(), "Goal reached at (%.2f, %.2f)", x_, y_);
            return;
        }

        VelocityCommand cmd = computeVelocity(distance, angle_error, params_);

        geometry_msgs::msg::Twist twist;
        twist.linear.x = cmd.linear;
        twist.angular.z = cmd.angular;
        cmd_pub_->publish(twist);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "pos=(%.2f, %.2f) yaw=%.1f deg | distance=%.2f angle_error=%.1f deg | v=%.2f w=%.2f",
            x_, y_, yaw_ * 180.0 / M_PI,
            distance, angle_error * 180.0 / M_PI, cmd.linear, cmd.angular);
    }

    void publishStop()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist());  // 所有字段默认为 0
    }

    // ===== 成员变量 =====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    bool has_odom_ = false;

    double goal_x_;
    double goal_y_;
    double goal_tolerance_;
    bool reached_ = false;
    ControllerParams params_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // 节点存在变量里，保证 rclcpp::shutdown() 执行停车回调时节点还活着
    auto node = std::make_shared<GoToGoal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
