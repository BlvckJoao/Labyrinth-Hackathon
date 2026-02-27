#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "maze_robot/navigation_node.hpp"


NavigationNode::NavigationNode() : 
Node("navigation_node"), state_(RobotState::FORWARD), 
front_distance_(10.0), right_distance_(10.0), left_distance_(10.0) {

        cmd_right_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor1/commands", 10);
        cmd_left_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor2/commands", 10);

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

        RobotState new_state;

        if (turning_) {

        std_msgs::msg::Float64MultiArray right_cmd;
        std_msgs::msg::Float64MultiArray left_cmd;

        if (turn_direction_ == RobotState::TURNING_RIGHT) {
                right_cmd.data = {5.0};
                left_cmd.data  = {-5.0};
        } else {
                right_cmd.data = {-5.0};
                left_cmd.data  = {5.0};
        }

        cmd_right_pub_->publish(right_cmd);
        cmd_left_pub_->publish(left_cmd);

        if (this->now() - turn_start_time_ > turn_duration_) {
                turning_ = false;
                state_ = RobotState::FORWARD;
        }

        return;
        }

        if (front_distance_ > 0.5) {
                new_state = RobotState::FORWARD;
        } else if (front_distance_ < 0.1) {
                new_state = RobotState::REAR;
        } else if (right_distance_ > left_distance_) {
                new_state = RobotState::TURNING_RIGHT;
        } else {
                new_state = RobotState::TURNING_LEFT;
        }

        if (new_state != state_) {
                state_ = new_state;
                braking_ = true;
                brake_start_time_ = this->now();
        }

        if (braking_) {

                right_cmd.data = {0.0};
                left_cmd.data  = {0.0};

                cmd_right_pub_->publish(right_cmd);
                cmd_left_pub_->publish(left_cmd);

                if (this->now() - brake_start_time_ > brake_duration_) {
                braking_ = false;
                }

                return;
        }

        switch (state_) {
                case RobotState::FORWARD:
                right_cmd.data = {7.5};
                left_cmd.data  = {7.5};
                break;

                case RobotState::TURNING_RIGHT:
                right_cmd.data = {5.0};
                left_cmd.data  = {-5.0};

                if (this->now() - turn_start_time_ > turn_duration_) {
                        turning_ = false;
                        state_ = RobotState::FORWARD;
                }

                break;

                case RobotState::TURNING_LEFT:
                right_cmd.data = {-5.0};
                left_cmd.data  = {5.0};
                if (this->now() - turn_start_time_ > turn_duration_) {
                        turning_ = false;
                        state_ = RobotState::FORWARD;
                }
                break;

                case RobotState::REAR:
                right_cmd.data = {-7.5};
                left_cmd.data  = {-7.5};
                break;
        }

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