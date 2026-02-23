#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "maze_robot/perception_node.hpp"


PerceptionNode::PerceptionNode() : Node("perception_node"), 
red_count_(0), green_count_(0), red_count(0)_ {

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
        } catch {
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
                red_count_++;
        }

        if(cv::countNonZero(green_mask) > 500) {
                green_mask_++;
        }

        if(cv::countNonZero(blue_mask) > 500) {
                blue_count_++;
        }

        std_msgs::msg::Int32 msg_red;
        std_msgs::msg::Int32 msg_green;
        std_msgs::msg::Int32 msg_blue;

        msg_red.data = red_count_;
        msg_green.data = green_count_;
        msg_blue.data = blue_count_;

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