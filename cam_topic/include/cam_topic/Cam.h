#ifndef __CAM__H__
#define __CAM__H__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class CAM : public rclcpp::Node {
public:
    CAM(int cam_number);
    CAM();
    ~CAM();
    void set_mode(bool mode);

private:
    void timer_callback();
    void divideFrame(cv::Mat &frame, cv::Mat &left, cv::Mat &right);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_l;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_r;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;

    int frame_width = 1280;
    int frame_height = 480;
    bool mode = true;
};

#endif
