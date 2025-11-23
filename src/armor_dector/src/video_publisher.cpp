#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {

        // 打开视频文件
        cap_.open(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR_STREAM(get_logger(), "无法打开视频文件: " << video_path_);
            rclcpp::shutdown();
            return;
        }

        // 创建图像发布者
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // 获取视频原始帧率
        frame_rate_ = cap_.get(cv::CAP_PROP_FPS);
        if (frame_rate_ <= 0) frame_rate_ = 30.0; // 回退到 30 FPS

        // 保护播放速度，避免除以0
        if (playback_speed_ <= 0.0) playback_speed_ = 1.0;

        // 根据播放速度调整定时器间隔（以毫秒为单位，至少为1ms）
        double effective_frame_rate = frame_rate_ * playback_speed_;
        double period_ms = 1000.0 / effective_frame_rate;
        int timer_interval = static_cast<int>(std::max(1.0, period_ms));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(timer_interval),
            std::bind(&VideoPublisher::timerCallback, this));

        RCLCPP_INFO_STREAM(get_logger(), "视频发布节点已启动");
        RCLCPP_INFO_STREAM(get_logger(), "视频路径: " << video_path_);
        RCLCPP_INFO(get_logger(), "原始帧率: %.2f FPS", frame_rate_);
        RCLCPP_INFO(get_logger(), "播放速度: %.2f x", playback_speed_);
        RCLCPP_INFO(get_logger(), "图像缩放: %.2f x", image_scale_);
    }

    ~VideoPublisher() {
        cap_.release();
    }

private:
    void timerCallback() {
        // 检查是否有订阅者，没有则减少处理
        if (image_pub_->get_subscription_count() == 0) {
            // 没有订阅者时，跳过几帧以节省资源
            static int skip_count = 0;
            if (skip_count++ % 5 != 0) {
                // 跳过帧但继续读取，保持视频位置更新
                cap_.grab();
                return;
            }
        }

        cv::Mat frame;
        if (!cap_.read(frame)) {
            // 视频播放完毕，重新开始
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        // 图像降采样以提高性能
        if (std::abs(image_scale_ - 1.0) > 1e-6) {
            cv::resize(frame, frame, cv::Size(), image_scale_, image_scale_, cv::INTER_AREA);
        }

        // 先构造 header，再生成消息
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera_link";

        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    std::string video_path_ = "/home/zheng/Desktop/test_ros/src/armor_dector/new.avi";
    double playback_speed_ = 1.0;
    double image_scale_ = 0.7;
    double frame_rate_ = 60.0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}