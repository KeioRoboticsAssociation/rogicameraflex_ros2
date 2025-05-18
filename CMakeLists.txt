cmake_minimum_required(VERSION 3.8)
project(rogicameraflex_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 実行ファイルの追加
add_executable(camera_flex_node
  src/camera_flex_node.cpp
  src/main.cpp
)

# リンクする ament パッケージ
ament_target_dependencies(camera_flex_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  yaml-cpp
)

# OpenCV ライブラリのリンク
target_link_libraries(camera_flex_node
  ${OpenCV_LIBRARIES}
)

# インストール設定
install(TARGETS camera_flex_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 著作権チェックと cpplint の自動スキップ
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
