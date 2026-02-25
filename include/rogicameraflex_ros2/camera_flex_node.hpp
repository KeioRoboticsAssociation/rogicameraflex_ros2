#ifndef ROGICAMERAFLEX_ROS2_CAMERA_FLEX_NODE_HPP_
#define ROGICAMERAFLEX_ROS2_CAMERA_FLEX_NODE_HPP_

#include "rogicameraflex_ros2/config_loader.hpp"
#include "rogicameraflex_ros2/camera_capture.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>

namespace rogicameraflex {

struct ActivePipeline {
  PipelineConfig config;
  image_transport::Publisher publisher;
};

class CameraFlexNode : public rclcpp::Node {
public:
  explicit CameraFlexNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CameraFlexNode() override;

private:
  void initialize();
  void captureLoop();

  NodeConfig config_;
  CameraCapture camera_;
  std::vector<ActivePipeline> pipelines_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;

  uint64_t frame_count_ = 0;
  uint32_t consecutive_capture_failures_ = 0;
  bool format_logged_ = false;
  bool initialized_ = false;
};

}  // namespace rogicameraflex

#endif  // ROGICAMERAFLEX_ROS2_CAMERA_FLEX_NODE_HPP_
