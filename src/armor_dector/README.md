# armor_detector (ROS2)

简短说明
- 这是一个用于检测装甲板灯条与配对生成装甲框的 ROS2 包（基于 OpenCV）。
- 支持蓝色/红色灯条切换、输出可视化图像与二值化图像、发布 RViz Marker。

文件结构（关键信息）
- src/armor_dector/src/：节点与实现文件（armor_detector_node.cpp、armor_detector.cpp、video_publisher.cpp 等）
- src/armor_dector/include/armor_detector/：头文件（armor_detector.hpp）
- src/config/ros__parameters:.yml：参数配置（注意文件名中的冒号，建议改为 `armor_params.yaml`）
- CMakeLists.txt / package.xml：包配置

主要话题（topic）
- 订阅（输入）
  - camera/image_raw — 输入 BGR 图像（来自摄像头或 video_publisher）
- 发布（输出）
  - armor/detection_result — 带标注的检测结果（image_transport）
  - armor/binary_image — 二值化图（mono8），可在 RViz 的 Image 中查看
  - armor/detection_markers — 装甲顶点的 MarkerArray（RViz）

快速构建与运行
1. 配置 ROS 环境并构建（在工作区根目录）
   ```
   source /opt/ros/<ros2-distro>/setup.bash
   cd /home/zheng/Desktop/test_ros
   colcon build --packages-select armor_detector
   source install/setup.bash
   ```
2. 运行节点（使用参数文件）
   ```
   ros2 run armor_detector armor_detector_node --ros-args --params-file /home/zheng/Desktop/test_ros/src/config/armor_params.yaml
   ```
3. 使用 video_publisher 发布本地视频作为输入（可选）
   ```
   ros2 run armor_detector video_publisher --ros-args -p video_path:="/home/zheng/Desktop/test_ros/src/armor_dector/new.avi"
   ```

参数调试（如何切换颜色）
- 在参数文件中修改 `target_color: "blue"` 或 `target_color: "red"`，或运行时覆盖：
  ```
  ros2 run armor_detector armor_detector_node --ros-args -p target_color:=red
  ```
- 常用可调参数：brightness_threshold, light_min_area, ANGLE_TOLERANCE（如已暴露），light_color_detect_extend_ratio 等（见参数文件）。

在 RViz 中查看二值图
- 启动 RViz2，添加 Image 显示项，订阅 `/armor/binary_image`，TransportHint 选 `raw`。
- Marker：订阅 `/armor/detection_markers` 可查看装甲四边（LINE_STRIP）。

VS Code includePath / 编译错误（rclcpp 找不到）
- 确保已 source ROS 环境后再打开 VS Code，或在 .vscode/c_cpp_properties.json 中加入：
  - /opt/ros/<ros2-distro>/include
  - ${workspaceFolder}/install/** 或 ${workspaceFolder}/src/**
- 示例：
  ```
  "/opt/ros/humble/include",
  "${workspaceFolder}/src/**",
  "${workspaceFolder}/install/**"
  ```

常见问题与排查
- 如果只看到“顶部灯条”或角度判断异常：检查 ANGLE 计算逻辑、灯条顶点顺序与 ANGLE_TOLERANCE 设置。
- 如果没有图像输出：用 `ros2 topic list` / `rqt_image_view` 确认话题有数据。
- 若二值化图为空：检查 target_color、brightness_threshold 与输入图像通道。

建议改进点（短）
- 将参数文件名中的冒号改为普通字符（如 `armor_params.yaml`）。
- 若检测耗时，可使用多线程 Executor 或将检测移入后台线程。
- 如果需要我可以：①把参数文件重命名并写入样例；②把 README 放到包根并提交；③帮你修改 includePath 文件。

如需我直接写入或修改 README 内容（或将参数文件重命名为 armor_params.yaml），告诉我即可。