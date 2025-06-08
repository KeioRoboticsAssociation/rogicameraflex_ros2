#ifndef ROGICAMERAFLEX_ROS2_CAMERA_FLEX_NODE_HPP_
#define ROGICAMERAFLEX_ROS2_CAMERA_FLEX_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <memory>

// 単一の画像処理操作を表現
struct Operation {
  std::string type;      // "crop", "resize", "color_convert", etc.
  YAML::Node params;     // パラメータ節（offset や encoding など）
};

// トピックごとのパイプライン情報
struct Pipeline {
  std::string name;
  std::string topic;
  std::vector<Operation> operations;
  image_transport::Publisher publisher;
};

class CameraFlexNode : public rclcpp::Node
{
public:
  explicit CameraFlexNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraFlexNode();

private:
  void captureLoop();
  cv::Mat applyOperations(const cv::Mat & frame, const std::vector<Operation> & ops);

  cv::VideoCapture cap_;
  std::vector<Pipeline> pipelines_;
  rclcpp::TimerBase::SharedPtr timer_;

  // カメラ設定
  std::string frame_id_;
  int width_;
  int height_;
  int fps_;
};

#endif  // ROGICAMERAFLEX_ROS2_CAMERA_FLEX_NODE_HPP_