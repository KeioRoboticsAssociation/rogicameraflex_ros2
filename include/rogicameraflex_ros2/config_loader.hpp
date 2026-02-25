#ifndef ROGICAMERAFLEX_ROS2_CONFIG_LOADER_HPP_
#define ROGICAMERAFLEX_ROS2_CONFIG_LOADER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rogicameraflex {

struct OperationConfig {
  std::string type;
  int x_offset = 0;
  int y_offset = 0;
  int width = 0;
  int height = 0;
  std::string encoding;
};

struct PipelineConfig {
  std::string name;
  std::string topic;
  std::vector<OperationConfig> operations;
};

struct CameraConfig {
  int device = 0;
  std::string frame_id = "camera_frame";
  int width = 1920;
  int height = 1080;
  int fps = 30;
};

struct NodeConfig {
  CameraConfig camera;
  std::vector<PipelineConfig> pipelines;
  uint32_t queue_size = 10;
};

[[nodiscard]] std::optional<NodeConfig> load_config(const std::string& path,
    std::string& error_out);

}  // namespace rogicameraflex

#endif  // ROGICAMERAFLEX_ROS2_CONFIG_LOADER_HPP_
