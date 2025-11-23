#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace rm {

enum class ArmorType { SMALL, BIG };

struct LightBar {
  cv::Point2f vertices[4];
  cv::Point2f center;
  float length;
  float width;
  float angle; // 添加角度成员变量
};

struct Armor {
  cv::Point2f vertices[4];
  cv::Point2f center;
  ArmorType type;
};

#include <mutex>

class ArmorDetector {
public:
  struct Params {
    // 基本参数
    int light_min_area = 20; // 灯条最小面积
    int brightness_threshold = 50; // 亮度阈值
    float big_armor_ratio = 2.5f; // 大装甲板比例阈值
    
    // 灯条筛选参数
    float light_max_ratio = 0.5f; // 灯条最大宽高比
    float light_contour_min_solidity = 0.5f; // 灯条最小凸度
    float light_color_detect_extend_ratio = 1.2f; // 灯条扩展比例
    
    // 装甲板匹配参数
    float light_max_angle_diff_ = 10.0f; // 灯条最大角度差(度)
    float light_max_height_diff_ratio_ = 0.3f; // 灯条最大高度差比率
    float light_max_y_diff_ratio_ = 0.2f; // 灯条最大y差比率
    float light_min_x_diff_ratio_ = 0.5f; // 灯条最小x差比率
    float armor_max_aspect_ratio_ = 4.0f; // 装甲板最大宽高比
    float armor_min_aspect_ratio_ = 1.0f; // 装甲板最小宽高比
    float armor_big_armor_ratio = 2.0f; // 大装甲板比例阈值

    // 新增：目标灯条颜色 ("blue" 或 "red")
    std::string target_color = "blue";
  };

  void init(const Params& params);
  void setParams(const Params& params);
  Params getParams() const;
  
  // 线程安全的检测方法
  std::vector<Armor> detect(const cv::Mat& image);
  std::vector<Armor> detectWithLightBars(const cv::Mat& image, std::vector<LightBar>& light_bars_out);
  std::vector<LightBar> detectLightBars(const cv::Mat& image);
  
  // 线程安全的可视化方法
  cv::Mat visualize(const cv::Mat& image, const std::vector<Armor>& armors);
  cv::Mat visualize(const cv::Mat& image, const std::vector<Armor>& armors, const std::vector<LightBar>& light_bars);

  // 新增：返回当前二值化图（线程安全的拷贝）
  cv::Mat getBinaryImage() const;

private:
  Params params_;
  cv::Mat hsv_image_;
  cv::Mat binary_image_;
  std::vector<std::vector<cv::Point>> contours_;
  mutable std::mutex params_mutex_; // 保护params_的互斥锁

  // 内部处理方法，使用局部参数副本以保证线程安全
  std::vector<LightBar> extractLightBars(const cv::Mat& image, const Params& local_params);
  std::vector<Armor> matchArmors(const std::vector<LightBar>& light_bars, const Params& local_params);
  bool isArmorPair(const LightBar& bar1, const LightBar& bar2, Armor& armor, const Params& local_params);
};

}  // namespace rm

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_