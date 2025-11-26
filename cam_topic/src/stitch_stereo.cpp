#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "cam_topic/Cam.h"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CAM>(0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
