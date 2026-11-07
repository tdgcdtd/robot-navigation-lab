#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("cmd_vel_subscriber");
    auto subscriber = 
        node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10,
            [](geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::cout << "linear.x = " << msg->linear.x << '\n';
            std::cout << "angular.z = " << msg->angular.z << '\n';
        }
    );

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}