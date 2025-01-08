cmake_minimum_required(VERSION 3.5)
project(usb_camera_package)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-permissive")
# 版本和依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 包含头文件路径
include_directories(include)

# 创建可执行文件
add_executable(camera_node src/camera_node.cpp)
add_executable(image_processor_node src/image_processor_node.cpp)

# 目标链接库
ament_target_dependencies(camera_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  camera_info_manager
)

ament_target_dependencies(image_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  camera_info_manager
)

# 安装
install(TARGETS
  camera_node
  image_processor_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
