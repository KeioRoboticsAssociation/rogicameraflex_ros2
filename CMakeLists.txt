cmake_minimum_required(VERSION 3.8)
project(rogicameraflex_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(camera_flex_node
  src/config_loader.cpp
  src/camera_capture.cpp
  src/image_pipeline.cpp
  src/camera_flex_node.cpp
  src/main.cpp
)

target_include_directories(camera_flex_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(camera_flex_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ament_index_cpp
)

target_link_libraries(camera_flex_node
  ${OpenCV_LIBRARIES}
  yaml-cpp::yaml-cpp
)

install(TARGETS camera_flex_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
