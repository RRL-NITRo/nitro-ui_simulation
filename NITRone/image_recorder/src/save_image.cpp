#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

class CameraSaver : public rclcpp::Node
{
public:
    CameraSaver()
    : Node("camera_saver")
    {
        // カメラトピック購読
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CameraSaver::imageCallback, this, std::placeholders::_1));

        // PS5コントローラ購読
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 10,
            std::bind(&CameraSaver::ps5Callback, this, std::placeholders::_1));

        save_dir_ = std::string(std::getenv("HOME")) + "/NITRo_WRS2025/wrs_ws/src/nitro_simulation/saved_images";
        std::filesystem::create_directories(save_dir_);

        RCLCPP_INFO(this->get_logger(), "CameraSaver Node started. Press PS5 HOME button to save image.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg)
    {
        if (msg->home) {
            saveImage();
        }
    }

    void saveImage()
    {
        if (latest_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image received yet!");
            return;
        }

        // 現在時刻からファイル名を作成
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".png";
        std::string filename = save_dir_ + "/" + oss.str();

        cv::imwrite(filename, latest_image_);
        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;

    cv::Mat latest_image_;
    std::string save_dir_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSaver>());
    rclcpp::shutdown();
    return 0;
}
