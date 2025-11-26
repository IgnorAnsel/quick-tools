#ifndef __CAM__H__
#define __CAM__H__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class CAM : public rclcpp::Node {
public:
    CAM(int cam_number) : Node("cam") {
        pub_l = this->create_publisher<sensor_msgs::msg::Image>("camera_l", 10);
        pub_r = this->create_publisher<sensor_msgs::msg::Image>("camera_r", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CAM::timer_callback, this));
        cap.open(cam_number, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FPS, 60);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap >> frame;
        cv::Mat left, right;
        divideFrame(frame, left, right);
        sensor_msgs::msg::Image msg_l, msg_r;
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left).toImageMsg(msg_l);
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right).toImageMsg(msg_r);
        pub_l->publish(msg_l);
        pub_r->publish(msg_r);
    }
    void divideFrame(cv::Mat &frame, cv::Mat &left, cv::Mat &right) {
        left = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
        right = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
    }
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_l;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_r;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
};

#endif
