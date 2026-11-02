#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iomanip>
#include <sstream>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>

namespace
{

bool directoryExists(const std::string & path)
{
  if (path.empty() || path == ".") {
    return true;
  }

  struct stat info = {};
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

}  // namespace

class PcdWriter : public rclcpp::Node
{
public:
  PcdWriter()
  : rclcpp::Node("pcd_writer"),
    save_once_(false),
    binary_(true),
    file_index_(0)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_cloud");
    directory_ = declare_parameter<std::string>("directory", ".");
    file_prefix_ = declare_parameter<std::string>("file_prefix", "cloud");
    save_once_ = declare_parameter<bool>("save_once", false);
    binary_ = declare_parameter<bool>("binary", true);
    file_index_ = declare_parameter<int>("start_index", 0);

    if (!directoryExists(directory_)) {
      RCLCPP_FATAL(get_logger(), "pcd_writer: directory '%s' does not exist", directory_.c_str());
      throw std::runtime_error("output directory not found");
    }

    RCLCPP_INFO(
      get_logger(),
      "pcd_writer: subscribing %s, writing to %s/%s_XXXX.pcd (binary=%s, save_once=%s)",
      input_topic_.c_str(), directory_.c_str(), file_prefix_.c_str(),
      binary_ ? "true" : "false", save_once_ ? "true" : "false");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PcdWriter::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    std::ostringstream oss;
    oss << directory_;
    if (!directory_.empty() && directory_ != "." && directory_.back() != '/') {
      oss << '/';
    }
    oss << file_prefix_ << '_' << std::setw(4) << std::setfill('0') << file_index_++ << ".pcd";

    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*msg, cloud);

    Eigen::Vector4f origin = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    int result = pcl::io::savePCDFile(oss.str(), cloud, origin, orientation, binary_);

    if (result != 0) {
      RCLCPP_ERROR(
        get_logger(), "pcd_writer: failed to save %s (error %d)", oss.str().c_str(), result);
      return;
    }

    RCLCPP_INFO(get_logger(), "pcd_writer: saved %s", oss.str().c_str());

    if (save_once_) {
      RCLCPP_INFO(get_logger(), "pcd_writer: save_once=true, shutting down");
      rclcpp::shutdown();
    }
  }

  std::string input_topic_;
  std::string directory_;
  std::string file_prefix_;
  bool save_once_;
  bool binary_;
  int file_index_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PcdWriter>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("pcd_writer"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
