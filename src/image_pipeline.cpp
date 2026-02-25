#include "rogicameraflex_ros2/image_pipeline.hpp"
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <algorithm>

namespace rogicameraflex {

PipelineResult apply_operations(const cv::Mat& frame,
    const std::vector<OperationConfig>& ops) {
  cv::Mat result = frame.clone();
  std::string encoding = (result.channels() == 1) ? "mono8" : "bgr8";

  for (const auto& op : ops) {
    if (op.type == "crop") {
      int x = std::max(0, std::min(op.x_offset, result.cols - 1));
      int y = std::max(0, std::min(op.y_offset, result.rows - 1));
      int w = std::min(op.width, result.cols - x);
      int h = std::min(op.height, result.rows - y);

      if (w > 0 && h > 0) {
        result = result(cv::Rect(x, y, w, h)).clone();
      }

    } else if (op.type == "resize") {
      if (op.width > 0 && op.height > 0) {
        cv::resize(result, result, cv::Size(op.width, op.height));
      }

    } else if (op.type == "color_convert") {
      if (op.encoding == "HSV" && result.channels() == 3) {
        cv::cvtColor(result, result, cv::COLOR_BGR2HSV);
        encoding = "8UC3";
      } else if (op.encoding == "GRAY" && result.channels() == 3) {
        cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
        encoding = "mono8";
      } else if (op.encoding != "HSV" && op.encoding != "GRAY") {
        RCLCPP_WARN(rclcpp::get_logger("image_pipeline"),
            "Unknown color_convert encoding: '%s', skipping", op.encoding.c_str());
      }

    } else {
      RCLCPP_WARN(rclcpp::get_logger("image_pipeline"),
          "Unknown operation type: '%s', skipping", op.type.c_str());
    }
  }

  return {result, encoding};
}

}  // namespace rogicameraflex
