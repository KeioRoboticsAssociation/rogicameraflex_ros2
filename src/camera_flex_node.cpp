#include "rogicameraflex_ros2/camera_flex_node.hpp"
#include "rogicameraflex_ros2/image_pipeline.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cinttypes>

namespace rogicameraflex {

static constexpr int kLogIntervalFrames = 30;
static constexpr int kMinReportedFps = 10;
static constexpr int kInitDelayMs = 100;
static constexpr int kInitialLogFrames = 3;
static constexpr int kErrorThrottleMs = 5000;
static constexpr uint32_t kMaxConsecutiveCaptureFailures = 150;

CameraFlexNode::CameraFlexNode(const rclcpp::NodeOptions& options)
    : Node("rogicameraflex_ros2", options) {
  // Resolve default config via installed share directory
  std::string default_path;
  try {
    default_path = ament_index_cpp::get_package_share_directory("rogicameraflex_ros2")
        + "/config/config.yaml";
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(),
        "Could not resolve package share directory: %s", e.what());
    default_path = "";
  }

  this->declare_parameter<std::string>("config_file", default_path);
  std::string config_file;
  this->get_parameter("config_file", config_file);

  RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

  std::string error;
  auto result = load_config(config_file, error);
  if (!result) {
    throw std::runtime_error("Config load failed: " + error);
  }
  config_ = std::move(*result);

  // Open camera
  if (!camera_.open(config_.camera.device, config_.camera.width,
                    config_.camera.height, config_.camera.fps)) {
    throw std::runtime_error(
        "Failed to open camera device: " + std::to_string(config_.camera.device));
  }

  auto cam_info = camera_.info();
  RCLCPP_INFO(this->get_logger(),
      "Camera opened. Resolution: %dx%d, FPS: %d (requested: %d)",
      cam_info.actual_width, cam_info.actual_height,
      cam_info.actual_fps, config_.camera.fps);
  RCLCPP_INFO(this->get_logger(), "Camera FOURCC: %s", cam_info.fourcc_str.c_str());

  if (cam_info.actual_width != config_.camera.width
      || cam_info.actual_height != config_.camera.height) {
    RCLCPP_WARN(this->get_logger(),
        "Camera resolution mismatch: requested %dx%d, got %dx%d",
        config_.camera.width, config_.camera.height,
        cam_info.actual_width, cam_info.actual_height);
  }

  if (cam_info.actual_fps >= kMinReportedFps) {
    config_.camera.fps = cam_info.actual_fps;
  } else {
    RCLCPP_WARN(this->get_logger(),
        "Camera reported low FPS (%d), using requested FPS (%d) for timer",
        cam_info.actual_fps, config_.camera.fps);
  }

  // Build pipeline list (publishers created later in initialize())
  for (auto& pc : config_.pipelines) {
    pipelines_.push_back({pc, {}});
    RCLCPP_INFO(this->get_logger(), "Added pipeline: %s -> %s",
        pc.name.c_str(), pc.topic.c_str());
  }

  // Deferred init — ImageTransport needs shared_from_this()
  init_timer_ = this->create_wall_timer(std::chrono::milliseconds(kInitDelayMs),
      std::bind(&CameraFlexNode::initialize, this));
}

void CameraFlexNode::initialize() {
  if (initialized_) {
    return;
  }
  initialized_ = true;

  init_timer_->cancel();
  init_timer_.reset();

  image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  for (auto& pipe : pipelines_) {
    pipe.publisher = image_transport_->advertise(pipe.config.topic, config_.queue_size);
    RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s",
        pipe.config.topic.c_str());
  }

  if (config_.camera.fps <= 0) {
    config_.camera.fps = 30;
  }
  auto period = std::chrono::milliseconds(1000 / config_.camera.fps);
  timer_ = this->create_wall_timer(period,
      std::bind(&CameraFlexNode::captureLoop, this));

  RCLCPP_INFO(this->get_logger(), "Timer created with period: %d ms",
      1000 / config_.camera.fps);
  RCLCPP_INFO(this->get_logger(), "CameraFlexNode initialization complete");
}

CameraFlexNode::~CameraFlexNode() {
  RCLCPP_DEBUG(this->get_logger(), "Shutting down CameraFlexNode...");

  if (init_timer_) {
    init_timer_->cancel();
  }
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  pipelines_.clear();
  camera_.release();

  RCLCPP_DEBUG(this->get_logger(), "CameraFlexNode shutdown complete.");
}

void CameraFlexNode::captureLoop() {
  cv::Mat frame;
  if (!camera_.read(frame)) {
    ++consecutive_capture_failures_;
    if (consecutive_capture_failures_ >= kMaxConsecutiveCaptureFailures) {
      RCLCPP_ERROR(this->get_logger(),
          "Camera capture failed %u consecutive times, stopping capture",
          consecutive_capture_failures_);
      timer_->cancel();
      return;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kErrorThrottleMs,
        "Failed to capture frame (consecutive failures: %u)",
        consecutive_capture_failures_);
    return;
  }
  consecutive_capture_failures_ = 0;

  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Captured frame is empty");
    return;
  }

  if (!format_logged_) {
    RCLCPP_INFO(this->get_logger(),
        "Input frame info - Channels: %d, Type: %d, Size: %dx%d",
        frame.channels(), frame.type(), frame.cols, frame.rows);
    format_logged_ = true;
  }

  if (++frame_count_ % kLogIntervalFrames == 0) {
    RCLCPP_INFO(this->get_logger(), "Processed frame %" PRIu64 ": %dx%d",
        frame_count_, frame.cols, frame.rows);
  }

  for (auto& pipe : pipelines_) {
    try {
      auto result = apply_operations(frame, pipe.config.operations);

      std_msgs::msg::Header hdr;
      hdr.stamp = this->now();
      hdr.frame_id = config_.camera.frame_id;

      auto img_msg = cv_bridge::CvImage(hdr, result.encoding, result.image)
          .toImageMsg();
      pipe.publisher.publish(img_msg);

      if (frame_count_ <= kInitialLogFrames) {
        RCLCPP_INFO(this->get_logger(),
            "Published frame to %s (encoding: %s, size: %dx%d)",
            pipe.config.topic.c_str(), result.encoding.c_str(),
            result.image.cols, result.image.rows);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), kErrorThrottleMs,
          "Failed to publish on topic %s: %s",
          pipe.config.topic.c_str(), e.what());
    }
  }
}

}  // namespace rogicameraflex
