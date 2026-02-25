#include "rogicameraflex_ros2/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>

namespace rogicameraflex {

static constexpr uint32_t kMinQueueSize = 1;
static constexpr uint32_t kMaxQueueSize = 1000;
static constexpr uint32_t kDefaultQueueSize = 10;

static OperationConfig parse_operation(const YAML::Node& op_node,
    std::string& error_out) {
  OperationConfig op;
  op.type = op_node["type"].as<std::string>("");

  if (op.type == "crop") {
    op.x_offset = op_node["x_offset"].as<int>(0);
    op.y_offset = op_node["y_offset"].as<int>(0);
    op.width = std::max(0, op_node["width"].as<int>(0));
    op.height = std::max(0, op_node["height"].as<int>(0));
  } else if (op.type == "resize") {
    op.width = std::max(0, op_node["width"].as<int>(0));
    op.height = std::max(0, op_node["height"].as<int>(0));
  } else if (op.type == "color_convert") {
    op.encoding = op_node["encoding"].as<std::string>("");
    if (op.encoding != "HSV" && op.encoding != "GRAY") {
      error_out = "Unknown color_convert encoding: '" + op.encoding
          + "' (supported: HSV, GRAY)";
    }
  } else if (!op.type.empty()) {
    error_out = "Unknown operation type: '" + op.type
        + "' (supported: crop, resize, color_convert)";
  }

  return op;
}

std::optional<NodeConfig> load_config(const std::string& path, std::string& error_out) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    error_out = std::string("Failed to load YAML: ") + e.what();
    return std::nullopt;
  }

  NodeConfig cfg;

  // Camera
  if (root["camera"] && root["camera"].IsMap()) {
    auto cam = root["camera"];
    cfg.camera.device = cam["device"].as<int>(0);
    cfg.camera.frame_id = cam["frame_id"].as<std::string>("camera_frame");
    cfg.camera.width = cam["width"].as<int>(1920);
    cfg.camera.height = cam["height"].as<int>(1080);
    cfg.camera.fps = cam["fps"].as<int>(30);
    if (cfg.camera.fps <= 0) {
      cfg.camera.fps = 30;
    }
  }

  // Node parameters
  if (root["node_parameters"] && root["node_parameters"].IsMap()) {
    int raw_queue_size = root["node_parameters"]["queue_size"].as<int>(
        static_cast<int>(kDefaultQueueSize));
    if (raw_queue_size < static_cast<int>(kMinQueueSize)
        || raw_queue_size > static_cast<int>(kMaxQueueSize)) {
      cfg.queue_size = kDefaultQueueSize;
    } else {
      cfg.queue_size = static_cast<uint32_t>(raw_queue_size);
    }
  }

  // Pipelines
  if (!root["pipelines"] || !root["pipelines"].IsSequence()) {
    error_out = "Config missing 'pipelines' sequence";
    return std::nullopt;
  }

  for (std::size_t i = 0; i < root["pipelines"].size(); ++i) {
    auto pipe_node = root["pipelines"][i];
    PipelineConfig pc;
    pc.name = pipe_node["name"].as<std::string>("");
    pc.topic = pipe_node["topic"].as<std::string>("");

    if (pc.topic.empty()) {
      error_out = "Pipeline '" + pc.name + "' has empty topic";
      return std::nullopt;
    }

    if (pipe_node["operations"] && pipe_node["operations"].IsSequence()) {
      for (std::size_t j = 0; j < pipe_node["operations"].size(); ++j) {
        auto op = parse_operation(pipe_node["operations"][j], error_out);
        if (!error_out.empty()) {
          return std::nullopt;
        }
        pc.operations.push_back(std::move(op));
      }
    }

    cfg.pipelines.push_back(std::move(pc));
  }

  return cfg;
}

}  // namespace rogicameraflex
