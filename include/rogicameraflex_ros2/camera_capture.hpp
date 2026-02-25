#ifndef ROGICAMERAFLEX_ROS2_CAMERA_CAPTURE_HPP_
#define ROGICAMERAFLEX_ROS2_CAMERA_CAPTURE_HPP_

#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <string>

namespace rogicameraflex {

struct CameraInfo {
  int actual_width = 0;
  int actual_height = 0;
  int actual_fps = 0;
  std::string fourcc_str;
};

class CameraCapture {
public:
  [[nodiscard]] bool open(int device_id, int width, int height, int fps);
  [[nodiscard]] bool read(cv::Mat& frame);
  [[nodiscard]] bool is_opened() const;
  [[nodiscard]] CameraInfo info() const;
  void release();

private:
  cv::VideoCapture cap_;
};

}  // namespace rogicameraflex

#endif  // ROGICAMERAFLEX_ROS2_CAMERA_CAPTURE_HPP_
