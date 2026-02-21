#ifndef _NAVIGATION_NODE_HPP_
#define _NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>

class NavigationNode : public rclcpp::Node {
        public:
                NavigationNode();

        private:
                void timer_callback();
                void scan_callback(const std::sensor_msgs::msg::Range::SharedPtr msg);

                rclcpp::Publisher<std_msgs::msg::FLoat64MultiArray>::SharedPtr cmd_right_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_left_pub_;

                rclcpp::Subscriber<sensor_msgs::msg::Range>::SharedPtr scan_front_sub_;
                rclcpp::Subscriber<sensor_msgs::msg::Range>::SharedPtr scan_right_sub_;
                rclcpp::Subscriber<sensor_msgs::msg::Range>::SharedPtr scan_left_sub_;
                
                rclcpp::TimerBase::SharedPtr timer_;

                enum class RobotState {
                        FOWARD,
                        TURNING_RIGHT,
                        TURNING_LEFT
                };

                RobotState state_;
                float front_distance_, right_distance_, left_distance_;
};

#endif
