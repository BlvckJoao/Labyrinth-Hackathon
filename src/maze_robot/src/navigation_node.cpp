#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/range.hpp>

class NavigationNode : public rclcpp::Node {
        public:
                NavigationNode() : 
                Node("navigation_node"), state_(RobotState::FOWARD), 
                front_distance_(10.0), right_distance_(10.0), left_distance_(10.0) {

                        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_vel", 10)
                        scan_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/scan", 10, 
                                std::bind(&NavigationNode::scan_callback, this, std::placeholders::_1));

                        timer_ = this->create_wall_timer(
                                std::chrono::milliseconds(100);
                                std::bind(&NavigationNode::timer_callback, this));
                        }

        private:
                void timer_callback() {
                        std_msgs::msg::Float64MultiArray cmd;

                        if (front_distance_ > 0.5) {
                                state_ = RobotState::FOWARD;
                        } else if (right_distance > left_distance) {
                                state_ = RobotState::TURNING_RIGHT;
                        } else {
                                state_ = RobotState::TURNING_LEFT;
                        }
                        }

                        switch (state_) {
                                case RobotState::FOWARD:
                                        break;

                                case RobotState::TURNING_RIGHT:
                                        break;
                                
                                case RobotState::TURNING_LEFT:
                                        break;
                        }

                        cmd_pub_->publish(cmd)
                }

                void scan_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
                        
                }

                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
                rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){

        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<NavigationNode>());
        rclcpp::shutdown();
        return 0;
}