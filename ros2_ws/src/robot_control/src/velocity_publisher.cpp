#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("velocity_publisher");
    auto publisher = 
        node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(500),
        [publisher]()
        {
            geometry_msgs::msg::Twist msg;

            msg.linear.x = 1.0;
            msg.angular.z = 0.5;

            publisher->publish(msg);
        }
    );

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}