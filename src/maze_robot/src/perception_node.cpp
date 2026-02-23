#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "maze_robot/perception_node.hpp"


PerceptionNode::PerceptionNode() : Node("perception_node"), 
red_counter_(0), green_counter_(0), blue_counter_(0) {

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/img_raw", 10,
                std::bind(&PerceptionNode::image_callback, this, std::placeholders::_1));

        red_pub_ = this->create_publisher<std_msgs::msg::Int32>("/colour_counts/red", 10);
        green_pub_ = this->create_publisher<std_msgs::msg::Int32>("/colour_counts/green", 10);
        blue_pub_ = this->create_publisher<std_msgs::msg::Int32>("/colour_counts/blue", 10);

}

void PerceptionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){

        cv::Mat frame;

        try {
                frame = cv::bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::cv_bridge_exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error");
                return;
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red_mask1, red_mask2 ,green_mask , blue_mask;

        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10,255,255), red_mask1);
        cv::inRange(hsv, cv::Scalar(170,120,70), cv::Scalar(180,255,255), red_mask2);
        cv::Mat red_mask = red_mask1 | red_mask2;

        cv::inRange(hsv, cv::Scalar(35, 100, 100), cv::Scalar(85,255,255), green_mask);

        cv::inRange(hsv, cv::Scalar(100, 150, 0), cv::Scalar(140,255,255), blue_mask);

        if(cv::countNonZero(red_mask) > 500) {
                red_counter_++;
        }

        if(cv::countNonZero(green_mask) > 500) {
                green_counter_++;
        }

        if(cv::countNonZero(blue_mask) > 500) {
                blue_counter_++;
        }

        std_msgs::msg::Int32 msg_red;
        std_msgs::msg::Int32 msg_green;
        std_msgs::msg::Int32 msg_blue;

        msg_red.data = red_counter_;
        msg_green.data = green_counter_;
        msg_blue.data = blue_counter_;

        red_pub_->publish(msg_red);
        green_pub_->publish(msg_green);
        blue_pub_->publish(msg_blue);
        
}

int main(int argc, char** argv){

        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<PerceptionNode>());
        rclcpp::shutdown();
        return 0;
}