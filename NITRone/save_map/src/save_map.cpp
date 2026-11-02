#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ps5_msgs/msg/ps5_joy.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

class CloudSaver : public rclcpp::Node
{
public:
    CloudSaver()
    : Node("cloud_saver")
    {
        // RTAB-Mapのcloud_map購読
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_map", 10,
            std::bind(&CloudSaver::cloudCallback, this, std::placeholders::_1));

        // PS5コントローラ購読
        ps5_sub_ = this->create_subscription<ps5_msgs::msg::PS5Joy>(
            "/ps5_joy", 10,
            std::bind(&CloudSaver::ps5Callback, this, std::placeholders::_1));

        save_dir_ = std::string(std::getenv("HOME")) + "/NITRo_WRS2025/wrs_ws/src/nitro_simulation/saved_cloud_maps";
        std::filesystem::create_directories(save_dir_);

        RCLCPP_INFO(this->get_logger(), "CloudSaver Node started. Press PS5 commandL button to save RTAB-Map cloud.");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_cloud_ = *msg;  // 最新点群を保持
    }

    void ps5Callback(const ps5_msgs::msg::PS5Joy::SharedPtr msg)
    {
        if (msg->command_l) {
            savePointCloud();
        }
    }

    void savePointCloud()
    {
        if (latest_cloud_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "No point cloud received yet!");
            return;
        }

        // ROSメッセージ → PCL (XYZRGB) に変換
        pcl::PointCloud<pcl::PointXYZRGB> cloud_pcl;
        pcl::fromROSMsg(latest_cloud_, cloud_pcl);

        if (cloud_pcl.empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty!");
            return;
        }

        // 現在時刻を使ってファイル名を作成
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".pcd";
        std::string filename = save_dir_ + "/" + oss.str();

        // バイナリ形式で保存（ASCIIより高速）
        if (pcl::io::savePCDFileBinary(filename, cloud_pcl) == 0) {
            RCLCPP_INFO(this->get_logger(), "Saved RTAB-Map cloud: %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ps5_msgs::msg::PS5Joy>::SharedPtr ps5_sub_;

    sensor_msgs::msg::PointCloud2 latest_cloud_;
    std::string save_dir_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudSaver>());
    rclcpp::shutdown();
    return 0;
}
