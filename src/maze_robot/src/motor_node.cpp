#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "maze_robot/motor_node.hpp"

MotorNode::MotorNode() : Node("motor_node")
{
    // Subscribers
    right_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/cmd_vel/right", 10,
        std::bind(&MotorNode::right_cmd_callback, this, std::placeholders::_1));

    left_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/cmd_vel/left", 10,
        std::bind(&MotorNode::left_cmd_callback, this, std::placeholders::_1));

    // Publishers
    motor1_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motor1/commands", 10);

    motor2_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/motor2/commands", 10);
}

void MotorNode::right_cmd_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    motor1_pub_->publish(*msg);
}

void MotorNode::left_cmd_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    motor2_pub_->publish(*msg);
}