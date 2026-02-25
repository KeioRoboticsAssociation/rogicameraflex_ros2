#ifndef ROGICAMERAFLEX_ROS2_IMAGE_PIPELINE_HPP_
#define ROGICAMERAFLEX_ROS2_IMAGE_PIPELINE_HPP_

#include "rogicameraflex_ros2/config_loader.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace rogicameraflex {

struct PipelineResult {
  cv::Mat image;
  std::string encoding;  // "bgr8", "mono8", "8UC3" (for HSV)
};

[[nodiscard]] PipelineResult apply_operations(const cv::Mat& frame,
    const std::vector<OperationConfig>& ops);

}  // namespace rogicameraflex

#endif  // ROGICAMERAFLEX_ROS2_IMAGE_PIPELINE_HPP_
