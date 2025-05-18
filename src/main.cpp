#include "rclcpp/rclcpp.hpp"
#include "rogicameraflex_ros2/camera_flex_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraFlexNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}