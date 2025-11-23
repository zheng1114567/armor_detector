#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "armor_detector/armor_detector.hpp"

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() : Node("armor_detector_node") {
        declareParameters();
        initDetector();

        // 新增：二值图发布器
        binary_pub_ = create_publisher<sensor_msgs::msg::Image>("armor/binary_image", 5);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { imageCallback(msg); });
        
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "armor/detection_markers", 10);
        
        result_pub_ = image_transport::create_publisher(this, "armor/detection_result");
        
        always_publish_result_ = get_parameter("always_publish_result").as_bool();
        
        param_cb_ = add_on_set_parameters_callback(
            [this](const auto& params) { return onParamChange(params); });
        
        RCLCPP_INFO(get_logger(), "装甲板检测节点已启动");
        RCLCPP_INFO(get_logger(), "识别结果将发布到话题: armor/detection_result");
    }

private:
    void declareParameters() {
        // 基本参数
        declare_parameter<int>("light_min_area", 20); // 灯条最小面积
        declare_parameter<int>("brightness_threshold", 50); // 亮度阈值
        declare_parameter<float>("big_armor_ratio", 2.5f); // 大装甲板比例阈值
        declare_parameter<bool>("always_publish_result", true);
        // 新增：目标颜色参数
        declare_parameter<std::string>("target_color", "blue");
        
        // 灯条筛选参数
        declare_parameter<float>("light_max_ratio", 0.5f); // 灯条最大宽高比
        declare_parameter<float>("light_contour_min_solidity", 0.5f); // 灯条最小凸度
        declare_parameter<float>("light_color_detect_extend_ratio", 1.2f); // 灯条扩展比例
        
        // 装甲板匹配参数
        declare_parameter<float>("light_max_angle_diff_", 10.0f); // 灯条最大角度差(度)
        declare_parameter<float>("light_max_height_diff_ratio_", 0.3f); // 灯条最大高度差比率
        declare_parameter<float>("light_max_y_diff_ratio_", 0.2f); // 灯条最大y差比率
        declare_parameter<float>("light_min_x_diff_ratio_", 0.5f); // 灯条最小x差比率
        declare_parameter<float>("armor_max_aspect_ratio_", 4.0f); // 装甲板最大宽高比
        declare_parameter<float>("armor_min_aspect_ratio_", 1.0f); // 装甲板最小宽高比
        declare_parameter<float>("armor_big_armor_ratio", 2.0f); // 大装甲板比例阈值
    }
    
    void initDetector() {
        rm::ArmorDetector::Params params;
        // 基本参数
        params.light_min_area = static_cast<int>(get_parameter("light_min_area").as_int());
        params.brightness_threshold = static_cast<int>(get_parameter("brightness_threshold").as_int());
        params.big_armor_ratio = get_parameter("big_armor_ratio").as_double();
        // 读取颜色参数
        params.target_color = get_parameter("target_color").as_string();
        // 灯条筛选参数
        params.light_max_ratio = get_parameter("light_max_ratio").as_double();
        params.light_contour_min_solidity = get_parameter("light_contour_min_solidity").as_double();
        params.light_color_detect_extend_ratio = get_parameter("light_color_detect_extend_ratio").as_double();
        
        // 装甲板匹配参数
        params.light_max_angle_diff_ = get_parameter("light_max_angle_diff_").as_double();
        params.light_max_height_diff_ratio_ = get_parameter("light_max_height_diff_ratio_").as_double();
        params.light_max_y_diff_ratio_ = get_parameter("light_max_y_diff_ratio_").as_double();
        params.light_min_x_diff_ratio_ = get_parameter("light_min_x_diff_ratio_").as_double();
        params.armor_max_aspect_ratio_ = get_parameter("armor_max_aspect_ratio_").as_double();
        params.armor_min_aspect_ratio_ = get_parameter("armor_min_aspect_ratio_").as_double();
        params.armor_big_armor_ratio = get_parameter("armor_big_armor_ratio").as_double();
        
        detector_.init(params);
    }
    
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter>& parameters) {
        auto params = detector_.getParams();
        for(const auto& param : parameters) {
            // 基本参数
            if(param.get_name() == "light_min_area") params.light_min_area = static_cast<int>(param.as_int());
            else if(param.get_name() == "brightness_threshold") params.brightness_threshold = static_cast<int>(param.as_int());
            else if(param.get_name() == "big_armor_ratio") params.big_armor_ratio = param.as_double();
            else if(param.get_name() == "target_color") params.target_color = param.as_string();
            // 灯条筛选参数
            else if(param.get_name() == "light_max_ratio") params.light_max_ratio = param.as_double();
            else if(param.get_name() == "light_contour_min_solidity") params.light_contour_min_solidity = param.as_double();
            else if(param.get_name() == "light_color_detect_extend_ratio") params.light_color_detect_extend_ratio = param.as_double();
            
            // 装甲板匹配参数
            else if(param.get_name() == "light_max_angle_diff_") params.light_max_angle_diff_ = param.as_double();
            else if(param.get_name() == "light_max_height_diff_ratio_") params.light_max_height_diff_ratio_ = param.as_double();
            else if(param.get_name() == "light_max_y_diff_ratio_") params.light_max_y_diff_ratio_ = param.as_double();
            else if(param.get_name() == "light_min_x_diff_ratio_") params.light_min_x_diff_ratio_ = param.as_double();
            else if(param.get_name() == "armor_max_aspect_ratio_") params.armor_max_aspect_ratio_ = param.as_double();
            else if(param.get_name() == "armor_min_aspect_ratio_") params.armor_min_aspect_ratio_ = param.as_double();
            else if(param.get_name() == "armor_big_armor_ratio") params.armor_big_armor_ratio = param.as_double();
        }
        detector_.setParams(params);
        RCLCPP_INFO(get_logger(), "参数已更新");
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
      try {
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat processed_img = img;
        const int max_dim = 800;
        if (img.cols > max_dim || img.rows > max_dim) {
          double scale = std::min(static_cast<double>(max_dim) / img.cols,
                                  static_cast<double>(max_dim) / img.rows);
          cv::resize(img, processed_img, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        std::vector<rm::LightBar> light_bars;
        auto armors = detector_.detectWithLightBars(processed_img, light_bars);

        // 发布二值化图（如果存在）
        cv::Mat bin = detector_.getBinaryImage();
        if (!bin.empty()) {
          // 确保是单通道8位图
          if (bin.type() != CV_8UC1) {
            cv::Mat tmp;
            if (bin.channels() == 3) cv::cvtColor(bin, tmp, cv::COLOR_BGR2GRAY);
            else bin.convertTo(tmp, CV_8U);
            bin = tmp;
          }
          auto bin_msg = cv_bridge::CvImage(msg->header, "mono8", bin).toImageMsg();
          binary_pub_->publish(*bin_msg);
        }

        // 只在需要时记录日志
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
                           "处理图像: 原始尺寸=(%d,%d), 处理尺寸=(%d,%d)", 
                           img.cols, img.rows, processed_img.cols, processed_img.rows);
        
        // 检测装甲板，同时获取检测到的灯条
        

        // 减少日志输出频率
        if(armors.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                               "未检测到装甲板");
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                               "检测到 %zu 个装甲板", armors.size());
        }
        
        publishMarkers(armors, msg->header);
        
        // 生成带标记的图像，传入已检测的灯条避免重复计算
        cv::Mat result_img = detector_.visualize(processed_img, armors, light_bars);
        
        // 如果进行了缩放，将结果图像放大回原始尺寸
        if (!img.empty() && (img.cols != processed_img.cols || img.rows != processed_img.rows)) {
            cv::resize(result_img, result_img, img.size());
        }
        
        // 确保result_img不为空
        if(result_img.empty()) {
            RCLCPP_WARN(get_logger(), "可视化结果为空");
            result_img = img.clone(); // 如果为空，使用原始图像
        }
        
        // 只在有订阅者时发布结果图像，或强制发布（always_publish_result_）
        if (always_publish_result_ || result_pub_.getNumSubscribers() > 0) {
            result_pub_.publish(cv_bridge::CvImage(msg->header, "bgr8", result_img).toImageMsg());
            RCLCPP_DEBUG(get_logger(), "发布 result 图像");
        }
      } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "exception in imageCallback: %s", e.what());
      }
    }
    
    void publishMarkers(const std::vector<rm::Armor>& armors,
                       const std_msgs::msg::Header& header) {
        if(armors.empty()) return;
        
        visualization_msgs::msg::MarkerArray marker_array;
        for(size_t i = 0; i < armors.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "armors";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker.color.r = (armors[i].type == rm::ArmorType::BIG) ? 1.0f : 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            marker.scale.x = 0.02;
            
            for(const auto& v : armors[i].vertices) {
                geometry_msgs::msg::Point point;
                point.x = v.x;
                point.y = v.y;
                point.z = 0.0;
                marker.points.push_back(point);
            }
            marker.points.push_back(marker.points.front());
            marker_array.markers.push_back(marker);
        }
        markers_pub_->publish(marker_array);
    }

    rm::ArmorDetector detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    image_transport::Publisher result_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    bool always_publish_result_;

    // 新增成员
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_;
};
 
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}