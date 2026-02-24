#ifndef _PERCEPTION_NODE_HPP_
#define _PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class PerceptionNode : public rclcpp::Node {
        public:
                PerceptionNode();

        private:
                void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

                rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr red_pub_;
                rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr blue_pub_;
                rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr green_pub_;

                int red_counter_;
                int green_counter_;
                int blue_counter_;
};


#endif
