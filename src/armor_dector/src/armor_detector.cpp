#include "armor_detector/armor_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace rm {

void ArmorDetector::init(const Params& params) {
  std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = params;
}

void ArmorDetector::setParams(const Params& params) {
  std::lock_guard<std::mutex> lock(params_mutex_);
  params_ = params;
}

ArmorDetector::Params ArmorDetector::getParams() const {
  std::lock_guard<std::mutex> lock(params_mutex_);
  return params_;
}

// 角度调整方向
const float ANGLE_TO_UP = 90.0f;

const float ANGLE_TOLERANCE = 15.0f;

// 角度调整函数
void adjustRec(cv::RotatedRect& rect, float ) {
  float angle = rect.angle;
  cv::Size2f size = rect.size;
  
  // 确保宽小于高，保持灯条垂直
  if (size.width > size.height) {
    std::swap(size.width, size.height);
    angle += 90.0f;
  }
  
  // 调整角度使其在合适范围内
  if (angle < -45.0f) {
    angle += 90.0f;
    std::swap(size.width, size.height);
  }
  
  rect.size = size;
  rect.angle = angle;
}

std::vector<Armor> ArmorDetector::detect(const cv::Mat& image) {
  std::vector<LightBar> light_bars;
  return detectWithLightBars(image, light_bars);
}

std::vector<Armor> ArmorDetector::detectWithLightBars(const cv::Mat& image, std::vector<LightBar>& light_bars_out) {
  Params local_params;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    local_params = params_;
  }
  
  try {
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    
    cv::Mat grayImg;

    // 根据参数选择检测颜色：blue -> B - R ; red -> R - B
    if (channels.size() >= 3) {
      std::string color = local_params.target_color;
      for (auto &c : color) c = static_cast<char>(std::tolower(c));
      if (color == "red") {
        cv::subtract(channels[2], channels[0], grayImg); // red - blue
      } else {
        cv::subtract(channels[0], channels[2], grayImg); // blue - red (默认)
      }
    } else {
      // 回退：直接转为灰度
      cv::cvtColor(image, grayImg, cv::COLOR_BGR2GRAY);
    }

    cv::Mat binBrightImg;
    cv::threshold(grayImg, binBrightImg, local_params.brightness_threshold, 255, cv::THRESH_BINARY);

    // 定义形态学元素（保证 element 已定义并可复用）
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    // 1) 平滑去脉冲噪声
    cv::medianBlur(binBrightImg, binBrightImg, 3);

    // 根据图像尺寸调整核大小
    int k_small = std::max(3, (std::min(image.cols, image.rows) / 200) | 1);
    int k_large = std::max(3, (std::min(image.cols, image.rows) / 80) | 1);

    cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k_small, k_small));
    cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k_large, k_large));

    // 2) 小核 opening 去噪（断开细小噪点），大核 closing 填充灯条内部小孔
    cv::morphologyEx(binBrightImg, binBrightImg, cv::MORPH_OPEN, kernel_small);
    cv::morphologyEx(binBrightImg, binBrightImg, cv::MORPH_CLOSE, kernel_large);

    // 3) 可选：top-hat 从灰度图增强小亮条（在复杂背景下有用）
    cv::Mat tophat;
    cv::morphologyEx(grayImg, tophat, cv::MORPH_TOPHAT, kernel_small);
    cv::threshold(tophat, tophat, std::max(1, local_params.brightness_threshold/2), 255, cv::THRESH_BINARY);
    cv::bitwise_or(binBrightImg, tophat, binBrightImg);

    // 4) 连通域过滤：去掉太小的区域（依据 light_min_area）
    cv::Mat labels, stats, centroids;
    int nlabels = cv::connectedComponentsWithStats(binBrightImg, labels, stats, centroids);
    cv::Mat filtered = cv::Mat::zeros(binBrightImg.size(), CV_8UC1);
    for (int i = 1; i < nlabels; ++i) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area >= local_params.light_min_area) {
        filtered.setTo(255, (labels == i));
      }
    }
    binBrightImg = filtered.clone();

    // 5) 可选后处理：细化或膨胀以保证灯条连通性（根据经验调参）
    cv::dilate(binBrightImg, binBrightImg, kernel_small, cv::Point(-1,-1), 1);
    cv::erode(binBrightImg, binBrightImg, kernel_small, cv::Point(-1,-1), 1);

    cv::Mat local_binary_image = binBrightImg.clone();
    
    light_bars_out.reserve(20); // 预设合理的初始容量
    
    light_bars_out = extractLightBars(binBrightImg, local_params);
    
    if (light_bars_out.size() < 2) {
      cv::threshold(grayImg, binBrightImg, local_params.brightness_threshold / 2, 255, cv::THRESH_BINARY);
      cv::dilate(binBrightImg, binBrightImg, element, cv::Point(-1, -1), 2); // 两次膨胀
      local_binary_image = binBrightImg.clone();
      light_bars_out = extractLightBars(binBrightImg, local_params);
    }
    
    sort(light_bars_out.begin(), light_bars_out.end(), [](const LightBar& ld1, const LightBar& ld2) {
      return ld1.center.x < ld2.center.x;
    });
    
    std::vector<Armor> armors = matchArmors(light_bars_out, local_params);
    
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      binary_image_ = local_binary_image;
    }
    
    return armors;
  } catch (const std::exception& e) {
    // 异常处理
    std::cerr << "检测过程中出错: " << e.what() << std::endl;
    return std::vector<Armor>();
  }
}

std::vector<LightBar> ArmorDetector::extractLightBars(const cv::Mat& image, const Params& local_params) {
  std::vector<LightBar> light_bars;
  
  light_bars.reserve(20); // 预设合理的初始容量
  
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  if (contours.empty()) {
    return light_bars;
  }
  
  std::vector<std::vector<cv::Point>> filtered_contours;
  filtered_contours.reserve(contours.size());
  
  for (const auto& contour : contours) {
    float lightContourArea = cv::contourArea(contour);
    if (lightContourArea >= local_params.light_min_area) {
      filtered_contours.push_back(contour);
    }
  }
  
  for (const auto& contour : filtered_contours) {
    try {
      cv::RotatedRect lightRec = cv::fitEllipse(contour);
      
      adjustRec(lightRec, ANGLE_TO_UP);
      
      cv::Point2f verts_tmp[4];
      lightRec.points(verts_tmp);
      float dx1 = verts_tmp[1].x - verts_tmp[0].x;
      float dy1 = verts_tmp[1].y - verts_tmp[0].y;
      float dx2 = verts_tmp[2].x - verts_tmp[1].x;
      float dy2 = verts_tmp[2].y - verts_tmp[1].y;
      float len1 = std::hypot(dx1, dy1);
      float len2 = std::hypot(dx2, dy2);
      float long_dx = (len1 > len2) ? dx1 : dx2;
      float long_dy = (len1 > len2) ? dy1 : dy2;
      float angle_deg = std::abs(std::atan2(long_dy, long_dx) * 180.0f / CV_PI); // 0..180
      if (angle_deg > 90.0f) angle_deg = 180.0f - angle_deg; // 0..90
      float angle_to_vertical = std::abs(90.0f - angle_deg);
      if (angle_to_vertical > ANGLE_TOLERANCE) {
        continue;
      }
      
      float aspect_ratio = lightRec.size.width / lightRec.size.height;
      
      float lightContourArea = cv::contourArea(contour);
      float solidity = lightContourArea / lightRec.size.area();
      
      if (aspect_ratio > local_params.light_max_ratio || 
          solidity < local_params.light_contour_min_solidity) {
        continue;
      }
      
      lightRec.size.width *= local_params.light_color_detect_extend_ratio;
      lightRec.size.height *= local_params.light_color_detect_extend_ratio;
      
      LightBar light_bar;
      cv::Point2f vertices[4];
      lightRec.points(vertices);
      for (int i = 0; i < 4; ++i) {
        light_bar.vertices[i] = vertices[i];
      }
      light_bar.center = lightRec.center;
      light_bar.length = lightRec.size.height;
      light_bar.width = lightRec.size.width;
      light_bar.angle = lightRec.angle;
      
      light_bars.push_back(light_bar);
    } catch (...) {
      continue;
    }
  }
  
  return light_bars;
}
// 装甲板匹配函数
std::vector<Armor> ArmorDetector::matchArmors(const std::vector<LightBar>& light_bars, const Params& local_params) {
  std::vector<Armor> armors;
  
  armors.reserve(10); 
  
  if (light_bars.size() < 2) {
    return armors;
  }
  
  const size_t MAX_LIGHT_BARS = 50;
  size_t effective_count = std::min(light_bars.size(), MAX_LIGHT_BARS);
  
  for (size_t i = 0; i < effective_count; ++i) {
    size_t max_j = std::min(i + 5, effective_count);
    for (size_t j = i + 1; j < max_j; ++j) {
      Armor armor;
      if (isArmorPair(light_bars[i], light_bars[j], armor, local_params)) {
        armors.push_back(armor);
      }
    }
  }
  
  return armors;
}
// 判断是否为装甲板
bool ArmorDetector::isArmorPair(const LightBar& bar1, const LightBar& bar2, Armor& armor, const Params& local_params) {
  const LightBar& leftLight = (bar1.center.x < bar2.center.x) ? bar1 : bar2;
  const LightBar& rightLight = (bar1.center.x < bar2.center.x) ? bar2 : bar1;
  
  float angleDiff_ = std::abs(leftLight.angle - rightLight.angle);
  
  float LenDiff_ratio = std::abs(leftLight.length - rightLight.length) / 
                       std::max(leftLight.length, rightLight.length);
  
  if (angleDiff_ > local_params.light_max_angle_diff_ || 
      LenDiff_ratio > local_params.light_max_height_diff_ratio_) {
    return false;
  }
  
  float dis = cv::norm(leftLight.center - rightLight.center);
  
  float meanLen = (leftLight.length + rightLight.length) / 2.0f;
  
  float yDiff = std::abs(leftLight.center.y - rightLight.center.y);
  float yDiff_ratio = yDiff / meanLen;
  
  float xDiff = std::abs(leftLight.center.x - rightLight.center.x);
  float xDiff_ratio = xDiff / meanLen;
  
  float ratio = dis / meanLen;
  
  if (yDiff_ratio > local_params.light_max_y_diff_ratio_ || 
      xDiff_ratio < local_params.light_min_x_diff_ratio_ || 
      ratio > local_params.armor_max_aspect_ratio_ || 
      ratio < local_params.armor_min_aspect_ratio_) {
    return false;
  }
  
  int armorType = (ratio > local_params.armor_big_armor_ratio) ? 
                  static_cast<int>(ArmorType::BIG) : static_cast<int>(ArmorType::SMALL);
  
  float armor_height = meanLen;
  float armor_width = std::abs(leftLight.center.x - rightLight.center.x);
  cv::Point2f armor_center = (leftLight.center + rightLight.center) / 2.0f;
  float armor_angle = (leftLight.angle + rightLight.angle) / 2.0f;
  cv::RotatedRect rect(armor_center, cv::Size2f(armor_width, armor_height), armor_angle);
  
  cv::Point2f vertices[4];
  rect.points(vertices);
  
  std::vector<cv::Point2f> top_points, bottom_points;
  float y_threshold = armor_center.y;
  
  for (const auto& pt : vertices) {
    if (pt.y < y_threshold) {
      top_points.push_back(pt);
    } else {
      bottom_points.push_back(pt);
    }
  }
  
  if (top_points.size() != 2 || bottom_points.size() != 2) {
    armor.vertices[0] = vertices[0];
    armor.vertices[1] = vertices[1];
    armor.vertices[2] = vertices[2];
    armor.vertices[3] = vertices[3];
  } else {
    if (top_points[0].x > top_points[1].x) std::swap(top_points[0], top_points[1]);
    if (bottom_points[0].x > bottom_points[1].x) std::swap(bottom_points[0], bottom_points[1]);
    
    armor.vertices[0] = top_points[0];    // 左上
    armor.vertices[1] = top_points[1];    // 右上
    armor.vertices[2] = bottom_points[1]; // 右下
    armor.vertices[3] = bottom_points[0]; // 左下
  }
  
  armor.center = armor_center;
  armor.type = static_cast<ArmorType>(armorType);
  
  return true;
}


cv::Mat ArmorDetector::visualize(const cv::Mat& image, const std::vector<Armor>& armors, const std::vector<LightBar>& light_bars) {
  // 如果输入为空，返回空矩阵
  if (image.empty()) {
    return cv::Mat();
  }

  cv::Mat result = image.clone();

  // 绘制灯条（仅绘制接近竖直的灯条），使用顶点方向判断竖直性
  if (!light_bars.empty()) {
    for (const auto& light_bar : light_bars) {
      cv::Point2f v[4];
      for (int i = 0; i < 4; ++i) v[i] = light_bar.vertices[i];

      float dx1 = v[1].x - v[0].x;
      float dy1 = v[1].y - v[0].y;
      float dx2 = v[2].x - v[1].x;
      float dy2 = v[2].y - v[1].y;
      float len1 = std::hypot(dx1, dy1);
      float len2 = std::hypot(dx2, dy2);
      float long_dx = (len1 > len2) ? dx1 : dx2;
      float long_dy = (len1 > len2) ? dy1 : dy2;
      float angle_deg = std::abs(std::atan2(long_dy, long_dx) * 180.0f / CV_PI);
      if (angle_deg > 90.0f) angle_deg = 180.0f - angle_deg;
      float angle_to_vertical = std::abs(90.0f - angle_deg);

      if (angle_to_vertical > ANGLE_TOLERANCE) continue;

      std::vector<cv::Point> points;
      for (int i = 0; i < 4; ++i) points.push_back(light_bar.vertices[i]);
      points.push_back(light_bar.vertices[0]); // 闭合
      cv::polylines(result, points, true, cv::Scalar(255, 255, 0), 2);
    }
  }

  for (const auto& armor : armors) {
    // 大装甲黄，小装甲紫
    cv::Scalar color = (armor.type == ArmorType::BIG) ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 255);


    cv::line(result, armor.vertices[0], armor.vertices[3], color, 3);
    cv::line(result, armor.vertices[1], armor.vertices[2], color, 3);

    cv::line(result, armor.vertices[0], armor.vertices[1], color, 3);
    cv::line(result, armor.vertices[3], armor.vertices[2], color, 3);
  }

  return result;
}

// 新增：线程安全返回二值图拷贝
cv::Mat ArmorDetector::getBinaryImage() const {
  std::lock_guard<std::mutex> lock(params_mutex_);
  return binary_image_.empty() ? cv::Mat() : binary_image_.clone();
}
};  // namespace rm
