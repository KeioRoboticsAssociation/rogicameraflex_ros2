#include "rogicameraflex_ros2/image_pipeline.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace rogicameraflex {

static std::string determine_encoding(const cv::Mat& img,
    const std::vector<OperationConfig>& ops) {
  // Check if any operation converted to HSV
  for (const auto& op : ops) {
    if (op.type == "color_convert" && op.encoding == "HSV") {
      return "8UC3";
    }
  }
  if (img.channels() == 1) {
    return "mono8";
  }
  return "bgr8";
}

PipelineResult apply_operations(const cv::Mat& frame,
    const std::vector<OperationConfig>& ops) {
  cv::Mat result = frame.clone();

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
      } else if (op.encoding == "GRAY" && result.channels() == 3) {
        cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
      }
    }
  }

  return {result, determine_encoding(result, ops)};
}

}  // namespace rogicameraflex
