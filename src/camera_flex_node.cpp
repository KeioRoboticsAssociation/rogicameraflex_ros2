#include "rogicameraflex_ros2/camera_flex_node.hpp"

CameraFlexNode::CameraFlexNode(const rclcpp::NodeOptions & options)
: Node("rogicameraflex_ros2", options)
{
  // パラメータ: YAML 設定ファイルのパス
  std::string config_file;
  this->declare_parameter<std::string>("config_file",
    "/home/imanoob/ros2_ws/src/rogicameraflex_ros/config/config.yaml");
  this->get_parameter("config_file", config_file);

  // YAML 読み込み
  YAML::Node config = YAML::LoadFile(config_file);

  // カメラ設定解析
  auto cam = config["camera"];
  int device = cam["device"].as<int>();
  frame_id_    = cam["frame_id"].as<std::string>();
  width_       = cam["width"].as<int>();
  height_      = cam["height"].as<int>();
  fps_         = cam["fps"].as<int>(30);

  cap_.open(device);
  if (!cap_.isOpened()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open camera device: %d", device);
    rclcpp::shutdown();
    return;
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS,          fps_);

  // ノードパラメータ
  int queue_size = config["node_parameters"]["queue_size"].as<int>(10);

    // パイプライン設定解析
  image_transport::ImageTransport it(this->shared_from_this());
  YAML::Node pipelines_node = config["pipelines"];
  for (std::size_t i = 0; i < pipelines_node.size(); ++i) {
    YAML::Node pipe_node = pipelines_node[i];
    Pipeline p;
    p.name = pipe_node["name"].as<std::string>();
    p.topic = pipe_node["topic"].as<std::string>();

    // 各操作を読み込み
    YAML::Node ops_node = pipe_node["operations"];
    for (std::size_t j = 0; j < ops_node.size(); ++j) {
      YAML::Node op_node = ops_node[j];
      Operation op;
      op.type = op_node["type"].as<std::string>();
      op.params = op_node;
      p.operations.push_back(op);
    }

    p.publisher = it.advertise(p.topic, queue_size);
    pipelines_.push_back(std::move(p));
  }

  // タイマーで定期的にフレーム取得＆パブリッシュ
  auto period = std::chrono::milliseconds(1000 / fps_);
  timer_ = this->create_wall_timer((
    period, std::bind(&CameraFlexNode::captureLoop, this));
}

void CameraFlexNode::captureLoop()
{
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
    return;
  }

  for (auto & pipe : pipelines_) {
    cv::Mat processed = applyOperations(frame, pipe.operations);

    // CvImage から ROS メッセージへ
    std_msgs::msg::Header hdr;
    hdr.stamp = this->now();
    hdr.frame_id = frame_id_;
    auto img_msg = cv_bridge::CvImage(hdr,
        sensor_msgs::image_encodings::BGR8, processed)
      .toImageMsg();

    pipe.publisher.publish(img_msg);
  }
}

cv::Mat CameraFlexNode::applyOperations(
  const cv::Mat & frame, const std::vector<Operation> & ops)
{
  cv::Mat result = frame.clone();

  for (const auto & op : ops) {
    if (op.type == "crop") {
      int x = op.params["x_offset"].as<int>();
      int y = op.params["y_offset"].as<int>();
      int w = op.params["width"].as<int>();
      int h = op.params["height"].as<int>();
      result = result(cv::Rect(x, y, w, h)).clone();

    } else if (op.type == "resize") {
      int w = op.params["width"].as<int>();
      int h = op.params["height"].as<int>();
      cv::resize(result, result, cv::Size(w, h));

    } else if (op.type == "color_convert") {
      std::string enc = op.params["encoding"].as<std::string>();
      if (enc == "HSV") {
        cv::cvtColor(result, result, cv::COLOR_BGR2HSV);
      } else if (enc == "GRAY") {
        cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
      }
    }
    // 他の操作もここに追加可能
  }
  return result;
}