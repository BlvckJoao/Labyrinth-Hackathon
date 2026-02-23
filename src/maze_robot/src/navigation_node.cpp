#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "maze_robot/navigation_node.hpp"


NavigationNode::NavigationNode() : 
Node("navigation_node"), state_(RobotState::FOWARD), 
front_distance_(10.0), right_distance_(10.0), left_distance_(10.0) {

        cmd_right_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor1/commands", 10);
        cmd_left_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor2/commands", 10);

        scan_front_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/sensor_range/front", 10, 
                std::bind(&NavigationNode::scan_callback, this, std::placeholders::_1));
        scan_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/sensor_range/right", 10, 
                std::bind(&NavigationNode::scan_callback, this, std::placeholders::_1));
        scan_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/sensor_range/left", 10, 
                std::bind(&NavigationNode::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100);
                std::bind(&NavigationNode::timer_callback, this));
        }

void NavigationNode::timer_callback() {

        std_msgs::msg::Float64MultiArray right_cmd;
        std_msgs::msg::Float64MultiArray left_cmd;

        if (front_distance_ > 0.5) {
                state_ = RobotState::FOWARD;
        } else if (right_distance > left_distance) {
                state_ = RobotState::TURNING_RIGHT;
        } else {
                state_ = RobotState::TURNING_LEFT;
        }

        switch (state_) {
                case RobotState::FOWARD:
                        right_cmd.data = {5.0};
                        left_cmd.data = {5.0};
                        break;

                case RobotState::TURNING_RIGHT:
                        right_cmd.data = {5.0};
                        left_cmd.data = {-5.0};
                        break;
                
                case RobotState::TURNING_LEFT:
                        right_cmd.data = {-5.0};
                        left_cmd.data = {5.0};
                        break;
        }

        cmd_right_pub_->publish(right_cmd);
        cmd_left_pub_->publish(left_cmd);
}

void NodeNavigation::scan_callback_front(const sensor_msgs::msg::Range::SharedPtr msg) {
        front_distance_ = msg->range;
}


void NodeNavigation::scan_callback_right(const sensor_msgs::msg::Range::SharedPtr msg) {
        right_distance_ = msg->range;
}


void NodeNavigation::scan_callback_left(const sensor_msgs::msg::Range::SharedPtr msg) {
        left_distance_ = msg->range;
}

int main(int argc, char** argv){

        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<NavigationNode>());
        rclcpp::shutdown();
        return 0;
}