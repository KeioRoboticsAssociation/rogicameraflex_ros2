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
  
  // カメラの設定
  cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS,          fps_);
  
  // カメラの色空間情報を取得・表示
  double fourcc = cap_.get(cv::CAP_PROP_FOURCC);
  char fourcc_str[5] = {0};
  memcpy(fourcc_str, &fourcc, 4);
  
  // 実際の設定値を確認
  int actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
  int actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
  int actual_fps = cap_.get(cv::CAP_PROP_FPS);
  
  RCLCPP_INFO(this->get_logger(), 
    "Camera opened successfully. Resolution: %dx%d, FPS: %d (requested: %d)", 
    actual_width, actual_height, actual_fps, fps_);
  RCLCPP_INFO(this->get_logger(), 
    "Camera FOURCC format: %s", fourcc_str);
  
  // FPSが低すぎる場合は、設定値を使用
  if (actual_fps < 10) {
    RCLCPP_WARN(this->get_logger(), 
      "Camera reported low FPS (%d), using requested FPS (%d) for timer", 
      actual_fps, fps_);
  } else {
    fps_ = actual_fps;
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
  static bool format_logged = false;
  static bool bgr_test_done = false;  // BGR確認用フラグ
  
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
    return;
  }
  
  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Captured frame is empty");
    return;
  }
  
  // BGR形式確認用のテスト（最初の数フレームでのみ実行）
  if (!bgr_test_done && frame_count < 5) {
    RCLCPP_INFO(this->get_logger(), "=== BGR Format Verification Test ===");
    
    // フレーム情報の詳細表示
    RCLCPP_INFO(this->get_logger(), 
      "Raw frame info - Size: %dx%d, Channels: %d, Type: %d, Depth: %d", 
      frame.cols, frame.rows, frame.channels(), frame.type(), frame.depth());
    
    // カメラの内部フォーマット情報
    double fourcc = cap_.get(cv::CAP_PROP_FOURCC);
    char fourcc_str[5] = {0};
    memcpy(fourcc_str, &fourcc, 4);
    RCLCPP_INFO(this->get_logger(), "Camera FOURCC: %s (0x%08X)", fourcc_str, (uint32_t)fourcc);
    
    // サンプルピクセルの値を確認（中央付近）
    if (frame.channels() == 3 && frame.rows > 200 && frame.cols > 200) {
      int test_x = frame.cols / 2;
      int test_y = frame.rows / 2;
      cv::Vec3b pixel = frame.at<cv::Vec3b>(test_y, test_x);
      
      RCLCPP_INFO(this->get_logger(), 
        "Sample pixel at (%d,%d): [%d, %d, %d]", 
        test_x, test_y, pixel[0], pixel[1], pixel[2]);
      
      // RGB/BGR判定のためのヒューリスティック
      // 一般的に、自然画像では赤成分よりも青成分の方が小さい傾向にある
      // もしpixel[0]が青でpixel[2]が赤なら、通常pixel[0] < pixel[2]
      RCLCPP_INFO(this->get_logger(), 
        "Channel analysis: Ch0=%d, Ch1=%d, Ch2=%d", 
        pixel[0], pixel[1], pixel[2]);
      
      // 複数点でサンプリング
      int total_ch0 = 0, total_ch1 = 0, total_ch2 = 0;
      int sample_count = 0;
      
      for (int y = 50; y < frame.rows - 50; y += 100) {
        for (int x = 50; x < frame.cols - 50; x += 100) {
          cv::Vec3b p = frame.at<cv::Vec3b>(y, x);
          total_ch0 += p[0];
          total_ch1 += p[1];
          total_ch2 += p[2];
          sample_count++;
        }
      }
      
      if (sample_count > 0) {
        double avg_ch0 = (double)total_ch0 / sample_count;
        double avg_ch1 = (double)total_ch1 / sample_count;
        double avg_ch2 = (double)total_ch2 / sample_count;
        
        RCLCPP_INFO(this->get_logger(), 
          "Average pixel values across %d samples: Ch0=%.1f, Ch1=%.1f, Ch2=%.1f", 
          sample_count, avg_ch0, avg_ch1, avg_ch2);
        
        // BGR判定のヒント
        RCLCPP_INFO(this->get_logger(), 
          "Format hint: If this is BGR, Ch0=Blue, Ch1=Green, Ch2=Red");
        RCLCPP_INFO(this->get_logger(), 
          "Format hint: If this is RGB, Ch0=Red, Ch1=Green, Ch2=Blue");
      }
      
      // HSV変換テスト
      cv::Mat hsv_test;
      cv::cvtColor(frame, hsv_test, cv::COLOR_BGR2HSV);
      cv::Vec3b hsv_pixel = hsv_test.at<cv::Vec3b>(test_y, test_x);
      
      RCLCPP_INFO(this->get_logger(), 
        "BGR->HSV conversion result: H=%d, S=%d, V=%d", 
        hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]);
      
      // RGB->HSV変換との比較
      cv::Mat hsv_test_rgb;
      cv::cvtColor(frame, hsv_test_rgb, cv::COLOR_RGB2HSV);
      cv::Vec3b hsv_pixel_rgb = hsv_test_rgb.at<cv::Vec3b>(test_y, test_x);
      
      RCLCPP_INFO(this->get_logger(), 
        "RGB->HSV conversion result: H=%d, S=%d, V=%d", 
        hsv_pixel_rgb[0], hsv_pixel_rgb[1], hsv_pixel_rgb[2]);
      
      // 色相(Hue)の違いで判定
      int hue_diff = abs(hsv_pixel[0] - hsv_pixel_rgb[0]);
      if (hue_diff > 60) {  // 色相の差が大きい場合
        RCLCPP_INFO(this->get_logger(), 
          "Large hue difference (%d) detected - indicates format matters for color accuracy", 
          hue_diff);
      }
    }
    
    // テスト画像を保存（デバッグ用）
    if (frame_count == 1) {
      // 元画像を保存
      cv::imwrite("/tmp/debug_original.jpg", frame);
      
      // BGR->RGB変換して保存
      cv::Mat rgb_frame;
      cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);
      cv::imwrite("/tmp/debug_bgr2rgb.jpg", rgb_frame);
      
      // RGB->BGR変換して保存
      cv::Mat bgr_frame;
      cv::cvtColor(frame, bgr_frame, cv::COLOR_RGB2BGR);
      cv::imwrite("/tmp/debug_rgb2bgr.jpg", bgr_frame);
      
      RCLCPP_INFO(this->get_logger(), 
        "Debug images saved: /tmp/debug_original.jpg, /tmp/debug_bgr2rgb.jpg, /tmp/debug_rgb2bgr.jpg");
      RCLCPP_INFO(this->get_logger(), 
        "Compare these images to determine the correct input format");
    }
    
    // テスト完了フラグを設定
    if (frame_count >= 3) {
      bgr_test_done = true;
      RCLCPP_INFO(this->get_logger(), "=== BGR Format Verification Complete ===");
    }
  }
  // 最初のフレームで詳細な形式情報をログ出力
  if (!format_logged) {
    RCLCPP_INFO(this->get_logger(), 
      "Input frame info - Channels: %d, Type: %d, Size: %dx%d", 
      frame.channels(), frame.type(), frame.cols, frame.rows);
    
    std::string type_str;
    switch(frame.type()) {
      case CV_8UC1: type_str = "CV_8UC1 (GRAY)"; break;
      case CV_8UC3: type_str = "CV_8UC3 (BGR)"; break;
      case CV_8UC4: type_str = "CV_8UC4 (BGRA)"; break;
      default: type_str = "Unknown type: " + std::to_string(frame.type()); break;
    }
    RCLCPP_INFO(this->get_logger(), "Frame type: %s", type_str.c_str());
    
    // OpenCVのVideoCapture出力は通常BGRであることを明示
    RCLCPP_INFO(this->get_logger(), "OpenCV VideoCapture output is BGR format - no RGB2BGR conversion needed");
    
    format_logged = true;
  }
  
  // OpenCVのVideoCaptureは通常BGRで出力するため、変換不要
  cv::Mat bgr_frame;
  if (frame.channels() == 3) {
    // BGR形式として直接使用（RGB→BGR変換を削除）
    bgr_frame = frame.clone();
  } else if (frame.channels() == 1) {
    // グレースケールの場合はそのまま使用
    bgr_frame = frame.clone();
  } else {
    RCLCPP_WARN_ONCE(this->get_logger(), 
      "Unsupported channel count: %d, using frame as-is", frame.channels());
    bgr_frame = frame.clone();
  }
  
  // 10フレームごとにログ出力
  if (++frame_count % 30 == 0) {
    RCLCPP_INFO(this->get_logger(), 
      "Processed frame %d: %dx%d", frame_count, bgr_frame.cols, bgr_frame.rows);
  }

  for (auto & pipe : pipelines_) {
    try {
      cv::Mat processed = applyOperations(bgr_frame, pipe.operations);

      // CvImage から ROS メッセージへ（BGR形式で出力）
      std_msgs::msg::Header hdr;
      hdr.stamp = this->now();
      hdr.frame_id = frame_id_;
      
      std::string encoding;
      if (processed.channels() == 3) {
        encoding = sensor_msgs::image_encodings::BGR8;
      } else if (processed.channels() == 1) {
        encoding = sensor_msgs::image_encodings::MONO8;
      } else {
        RCLCPP_WARN_ONCE(this->get_logger(), 
          "Unexpected channel count %d, using BGR8 encoding", processed.channels());
        encoding = sensor_msgs::image_encodings::BGR8;
      }
      
      auto img_msg = cv_bridge::CvImage(hdr, encoding, processed).toImageMsg();
      pipe.publisher.publish(img_msg);
      
      // 最初の数フレームはログ出力
      if (frame_count <= 3) {
        RCLCPP_INFO(this->get_logger(), 
          "Published frame to %s (encoding: %s, size: %dx%d)", 
          pipe.topic.c_str(), encoding.c_str(), processed.cols, processed.rows);
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
  static bool hsv_conversion_logged = false;

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
        RCLCPP_DEBUG(this->get_logger(), 
          "Applied crop: (%d,%d) %dx%d", x, y, w, h);
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid crop region, skipping crop operation");
      }

    } else if (op.type == "resize") {
      int w = op.params["width"].as<int>();
      int h = op.params["height"].as<int>();
      if (w > 0 && h > 0) {
        cv::resize(result, result, cv::Size(w, h));
        RCLCPP_DEBUG(this->get_logger(), "Applied resize: %dx%d", w, h);
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid resize dimensions, skipping resize operation");
      }

    } else if (op.type == "color_convert") {
      std::string enc = op.params["encoding"].as<std::string>();
      
      if (enc == "HSV") {
        if (result.channels() == 3) {
          // BGRからHSVに変換（入力がsRGB→BGRに変換済みなので）
          cv::cvtColor(result, result, cv::COLOR_BGR2HSV);
          
          if (!hsv_conversion_logged) {
            RCLCPP_INFO(this->get_logger(), 
              "Applied BGR to HSV conversion. Result: %dx%d, %d channels", 
              result.cols, result.rows, result.channels());
            
            // サンプルピクセルでの変換確認（デバッグ用）
            if (result.rows > 100 && result.cols > 100) {
              cv::Vec3b hsv_pixel = result.at<cv::Vec3b>(100, 100);
              RCLCPP_INFO(this->get_logger(), 
                "Sample HSV pixel at (100,100): H=%d, S=%d, V=%d", 
                hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]);
            }
            hsv_conversion_logged = true;
          }
        } else {
          RCLCPP_WARN_ONCE(this->get_logger(), 
            "Cannot convert to HSV: input is not 3-channel (%d channels)", 
            result.channels());
        }
        
      } else if (enc == "GRAY") {
        if (result.channels() == 3) {
          cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
          // ROSのimage_transportのため、グレースケールのままにしておく
          RCLCPP_DEBUG(this->get_logger(), "Applied BGR to GRAY conversion");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "Image is already grayscale, skipping conversion");
        }
        
      } else {
        RCLCPP_WARN_ONCE(this->get_logger(), 
          "Unsupported color encoding: %s", enc.c_str());
      }
    } else {
      RCLCPP_WARN_ONCE(this->get_logger(), 
        "Unknown operation type: %s", op.type.c_str());
    }
  }
  
  return result;
}