#ifndef _MOTOR_NODE_HPP_
#define _MOTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class MotorNode : public rclcpp::Node {

public:
    MotorNode();

private:

    void right_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void left_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_cmd_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor2_pub_;
};


#endif
