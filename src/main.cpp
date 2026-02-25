#include <rclcpp/rclcpp.hpp>
#include "rogicameraflex_ros2/camera_flex_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rogicameraflex::CameraFlexNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rogicameraflex_ros2"),
        "Node initialization failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
