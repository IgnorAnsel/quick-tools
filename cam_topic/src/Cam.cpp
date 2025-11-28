#include "cam_topic/Cam.h"

CAM::CAM(int cam_number) : Node("cam") {
    if (mode) {
        pub_l = this->create_publisher<sensor_msgs::msg::Image>("camera_l", 10);
        pub_r = this->create_publisher<sensor_msgs::msg::Image>("camera_r", 10);
    } else
        pub = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CAM::timer_callback, this));
    cap.open(cam_number, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
}

CAM::CAM() : Node("cam") {
    if (mode) {
        pub_l = this->create_publisher<sensor_msgs::msg::Image>("camera_l", 10);
        pub_r = this->create_publisher<sensor_msgs::msg::Image>("camera_r", 10);
    } else
        pub = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CAM::timer_callback, this));
    cap.open(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
}

CAM::~CAM() {
    cap.release();
}

void CAM::set_mode(bool mode) {
    this->mode = mode;
}

void CAM::timer_callback() {
    cv::Mat frame;
    cap >> frame;
    if (mode) {
        cv::Mat left, right;
        divideFrame(frame, left, right);
        sensor_msgs::msg::Image msg_l, msg_r;
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left).toImageMsg(msg_l);
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right).toImageMsg(msg_r);
        pub_l->publish(msg_l);
        pub_r->publish(msg_r);
    } else {
        sensor_msgs::msg::Image msg;
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg(msg);
        pub->publish(msg);
    }
}

void CAM::divideFrame(cv::Mat &frame, cv::Mat &left, cv::Mat &right) {
    left = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
    right = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
}
