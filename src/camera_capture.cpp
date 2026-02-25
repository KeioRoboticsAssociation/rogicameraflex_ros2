#include "rogicameraflex_ros2/camera_capture.hpp"

namespace rogicameraflex {

bool CameraCapture::open(int device_id, int width, int height, int fps) {
  cap_.open(device_id, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    cap_.open(device_id);
    if (!cap_.isOpened()) {
      return false;
    }
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap_.set(cv::CAP_PROP_FPS, fps);

  return true;
}

bool CameraCapture::read(cv::Mat& frame) {
  return cap_.read(frame);
}

bool CameraCapture::is_opened() const {
  return cap_.isOpened();
}

CameraInfo CameraCapture::info() const {
  CameraInfo ci;
  ci.actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  ci.actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  ci.actual_fps = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));

  auto fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
  char buf[5] = {
    static_cast<char>(fourcc & 0xFF),
    static_cast<char>((fourcc >> 8) & 0xFF),
    static_cast<char>((fourcc >> 16) & 0xFF),
    static_cast<char>((fourcc >> 24) & 0xFF),
    '\0'
  };
  ci.fourcc_str = buf;

  return ci;
}

void CameraCapture::release() {
  if (cap_.isOpened()) {
    cap_.release();
  }
}

}  // namespace rogicameraflex
