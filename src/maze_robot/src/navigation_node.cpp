#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "maze_robot/navigation_node.hpp"


NavigationNode::NavigationNode() : 
Node("navigation_node"), state_(RobotState::FORWARD), 
front_distance_(10.0), right_distance_(10.0), left_distance_(10.0) {

        cmd_right_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_vel/right", 10);
        cmd_left_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_vel/left", 10);

        scan_front_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/sensor_range/front", 10, 
                std::bind(&NavigationNode::scan_callback_front, this, std::placeholders::_1));
        scan_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/sensor_range/right", 10, 
                std::bind(&NavigationNode::scan_callback_right, this, std::placeholders::_1));
        scan_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/sensor_range/left", 10, 
                std::bind(&NavigationNode::scan_callback_left, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&NavigationNode::timer_callback, this));
        }

void NavigationNode::timer_callback() {
        std_msgs::msg::Float64MultiArray right_cmd;
        std_msgs::msg::Float64MultiArray left_cmd;

        auto now = this->now();

        if (turning_) {

                if (turn_direction_ == RobotState::TURNING_RIGHT) {
                right_cmd.data = {4.0};
                left_cmd.data  = {-4.0};
                } else {
                right_cmd.data = {-4.0};
                left_cmd.data  = {4.0};
                }

                cmd_right_pub_->publish(right_cmd);
                cmd_left_pub_->publish(left_cmd);

                if (now - turn_start_time_ >= turn_duration_) {
                turning_ = false;
                }

                return;
        }

        if (moving_after_turn_) {

                right_cmd.data = {6.0};
                left_cmd.data  = {6.0};

                cmd_right_pub_->publish(right_cmd);
                cmd_left_pub_->publish(left_cmd);

                if (now - move_start_time_ >= move_duration_) {
                        moving_after_turn_ = false;
        }

        return;

        if (front_distance_ < 0.5) {

                if (right_distance_ > left_distance_) {
                turn_direction_ = RobotState::TURNING_RIGHT;
                } else {
                turn_direction_ = RobotState::TURNING_LEFT;
                }

                turning_ = true;
                turn_start_time_ = now;

                return;
        }

        right_cmd.data = {7.0};
        left_cmd.data  = {7.0};

        cmd_right_pub_->publish(right_cmd);
        cmd_left_pub_->publish(left_cmd);
}

void NavigationNode::scan_callback_front(const sensor_msgs::msg::Range::SharedPtr msg) {
        front_distance_ = msg->range;
}


void NavigationNode::scan_callback_right(const sensor_msgs::msg::Range::SharedPtr msg) {
        right_distance_ = msg->range;
}


void NavigationNode::scan_callback_left(const sensor_msgs::msg::Range::SharedPtr msg) {
        left_distance_ = msg->range;
}

int main(int argc, char** argv){

        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<NavigationNode>());
        rclcpp::shutdown();
        return 0;
}