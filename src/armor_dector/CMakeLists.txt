cmake_minimum_required(VERSION 3.8)
project(armor_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(armor_detector_node src/armor_detector_node.cpp src/armor_detector.cpp)

# 包含头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 链接库
ament_target_dependencies(armor_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  visualization_msgs
)
target_link_libraries(armor_detector_node ${OpenCV_LIBRARIES})

# 安装可执行文件
install(TARGETS
  armor_detector_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()