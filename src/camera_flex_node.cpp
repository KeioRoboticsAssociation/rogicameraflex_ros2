#include "rogicameraflex_ros2/camera_flex_node.hpp"

CameraFlexNode::CameraFlexNode(const rclcpp::NodeOptions & options)
: Node("rogicameraflex_ros2", options)
{
  // パラメータ: YAML 設定ファイルのパス
  std::string config_file;
  this->declare_parameter<std::string>("config_file",
    "/home/imanoob/ros2_ws/src/rogicameraflex_ros/config/config.yaml");
  this->get_parameter("config_file", config_file);

  RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

  // YAML 読み込み
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load config file: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // カメラ設定解析
  auto cam = config["camera"];
  int device = cam["device"].as<int>();
  frame_id_    = cam["frame_id"].as<std::string>();
  width_       = cam["width"].as<int>();
  height_      = cam["height"].as<int>();
  fps_         = cam["fps"].as<int>(30);

  // V4L2バックエンドを明示的に使用してGStreamer警告を回避
  cap_.open(device, cv::CAP_V4L2);
  if (!cap_.isOpened()) {
    // V4L2が失敗した場合、デフォルトバックエンドで再試行
    RCLCPP_WARN(this->get_logger(), "Failed to open camera with V4L2, trying default backend");
    cap_.open(device);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open camera device: %d", device);
      rclcpp::shutdown();
      return;
    }
  }
  
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS,          fps_);
  
  // 実際の設定値を確認
  int actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  int actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
  int actual_fps = cap_.get(cv::CAP_PROP_FPS);
  RCLCPP_INFO(this->get_logger(), 
    "Camera opened successfully. Resolution: %dx%d, FPS: %d (requested: %d)", 
    actual_width, actual_height, actual_fps, fps_);
  
  // FPSが低すぎる場合は、設定値を使用
  if (actual_fps < 10) {
    RCLCPP_WARN(this->get_logger(), 
      "Camera reported low FPS (%d), using requested FPS (%d) for timer", 
      actual_fps, fps_);
    // fps_ はそのまま使用（YAMLの設定値）
  } else {
    fps_ = actual_fps;  // カメラの実際のFPSを使用
  }

  // ノードパラメータ
  queue_size_ = config["node_parameters"]["queue_size"].as<int>(10);

  // パイプライン設定解析
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

    pipelines_.push_back(std::move(p));
    RCLCPP_INFO(this->get_logger(), "Added pipeline: %s -> %s", 
      pipelines_.back().name.c_str(), pipelines_.back().topic.c_str());
  }

  // 初期化タイマーを作成（100ms後に初期化）
  init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&CameraFlexNode::initialize, this));
}

void CameraFlexNode::initialize()
{
  if (initialized_) {
    return;
  }
  initialized_ = true;
  
  // 初期化タイマーを停止
  init_timer_->cancel();
  
  RCLCPP_INFO(this->get_logger(), "Starting initialization...");
  
  // ImageTransport を作成してパブリッシャーを初期化
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  
  RCLCPP_INFO(this->get_logger(), "Initializing %zu pipelines", pipelines_.size());
  
  for (auto & pipe : pipelines_) {
    pipe.publisher = it_->advertise(pipe.topic, queue_size_);
    RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", pipe.topic.c_str());
  }

  // タイマーで定期的にフレーム取得＆パブリッシュ
  auto period = std::chrono::milliseconds(1000 / fps_);
  timer_ = this->create_wall_timer(period, std::bind(&CameraFlexNode::captureLoop, this));
  
  RCLCPP_INFO(this->get_logger(), "Timer created with period: %d ms", 1000 / fps_);
  RCLCPP_INFO(this->get_logger(), "CameraFlexNode initialization complete");
}

CameraFlexNode::~CameraFlexNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down CameraFlexNode...");
  
  if (init_timer_) {
    init_timer_->cancel();
  }
  
  if (timer_) {
    timer_->cancel();
    timer_.reset();
    RCLCPP_INFO(this->get_logger(), "Timer reset.");
  }

  // Clear pipelines before releasing the camera
  // This ensures publishers are destroyed first
  pipelines_.clear();
  RCLCPP_INFO(this->get_logger(), "Pipelines cleared.");

  if (cap_.isOpened()) {
    cap_.release();
    RCLCPP_INFO(this->get_logger(), "Camera capture released.");
  }
  RCLCPP_INFO(this->get_logger(), "CameraFlexNode shutdown complete.");
}

void CameraFlexNode::captureLoop()
{
  static int frame_count = 0;
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
    return;
  }
  
  // フレームが正常に取得できたかチェック
  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Captured frame is empty");
    return;
  }
  
  // 10フレームごとにログ出力
  if (++frame_count % 10 == 0) {
    RCLCPP_INFO(this->get_logger(), 
      "Captured frame %d: %dx%d", frame_count, frame.cols, frame.rows);
  }

  for (auto & pipe : pipelines_) {
    // パブリッシャーが有効かチェック（初期化されていればpublishは安全）
    try {
      cv::Mat processed = applyOperations(frame, pipe.operations);

      // CvImage から ROS メッセージへ
      std_msgs::msg::Header hdr;
      hdr.stamp = this->now();
      hdr.frame_id = frame_id_;
      auto img_msg = cv_bridge::CvImage(hdr,
          sensor_msgs::image_encodings::BGR8, processed)
        .toImageMsg();

      pipe.publisher.publish(img_msg);
      
      // 最初の数フレームはログ出力
      if (frame_count <= 5) {
        RCLCPP_INFO(this->get_logger(), 
          "Published frame to %s", pipe.topic.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_ONCE(this->get_logger(), 
        "Failed to publish on topic %s: %s", pipe.topic.c_str(), e.what());
    }
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
      
      // 境界チェック
      x = std::max(0, std::min(x, result.cols - 1));
      y = std::max(0, std::min(y, result.rows - 1));
      w = std::min(w, result.cols - x);
      h = std::min(h, result.rows - y);
      
      if (w > 0 && h > 0) {
        result = result(cv::Rect(x, y, w, h)).clone();
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid crop region, skipping crop operation");
      }

    } else if (op.type == "resize") {
      int w = op.params["width"].as<int>();
      int h = op.params["height"].as<int>();
      if (w > 0 && h > 0) {
        cv::resize(result, result, cv::Size(w, h));
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid resize dimensions, skipping resize operation");
      }

    } else if (op.type == "color_convert") {
      std::string enc = op.params["encoding"].as<std::string>();
      if (enc == "HSV") {
        cv::cvtColor(result, result, cv::COLOR_BGR2HSV);
      } else if (enc == "GRAY") {
        cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
        // グレースケールの場合、BGR8として扱うために3チャンネルに戻す
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
      }
    }
    // 他の操作もここに追加可能
  }
  return result;
}