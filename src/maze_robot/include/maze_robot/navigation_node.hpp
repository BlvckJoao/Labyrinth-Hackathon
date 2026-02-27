#ifndef _NAVIGATION_NODE_HPP_
#define _NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>

class NavigationNode : public rclcpp::Node {
        public:
                NavigationNode();

        private:
                void timer_callback();

                void scan_callback_front(const sensor_msgs::msg::Range::SharedPtr msg);
                void scan_callback_right(const sensor_msgs::msg::Range::SharedPtr msg);
                void scan_callback_left(const sensor_msgs::msg::Range::SharedPtr msg);

                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_right_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_left_pub_;

                rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr scan_front_sub_;
                rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr scan_right_sub_;
                rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr scan_left_sub_;

                rclcpp::TimerBase::SharedPtr timer_;

                enum class RobotState {
                        FORWARD,
                        TURNING_RIGHT,
                        TURNING_LEFT,
                        REAR
                };

                bool braking_ = false;
                rclcpp::Time brake_start_time_;
                rclcpp::Duration brake_duration_{0, 200000000};

                RobotState state_;
                float front_distance_, right_distance_, left_distance_;
};

#endif
