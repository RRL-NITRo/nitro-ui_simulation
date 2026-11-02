#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace slam_stack
{

namespace
{
const char * kFieldNames[3] = {"x", "y", "z"};

bool has_xyz_fields(const sensor_msgs::msg::PointCloud2 & cloud)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto & field : cloud.fields) {
    if (field.name == "x") {
      has_x = true;
    } else if (field.name == "y") {
      has_y = true;
    } else if (field.name == "z") {
      has_z = true;
    }
  }
  return has_x && has_y && has_z;
}

sensor_msgs::msg::PointCloud2::SharedPtr normalize_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  rclcpp::Logger logger)
{
  if (!has_xyz_fields(input)) {
    RCLCPP_WARN(logger, "cloud_repacker: incoming cloud missing x/y/z fields - skipping");
    return nullptr;
  }

  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  output->header = input.header;
 output->height = input.height == 0 ? 1u : input.height;

  uint32_t width = input.width;
  if (width == 0 && input.point_step > 0 && !input.data.empty()) {
    width = static_cast<uint32_t>(input.data.size() / input.point_step);
  }
  output->width = width;
  if (output->width == 0) {
    RCLCPP_WARN(logger, "cloud_repacker: incoming cloud has zero width - skipping");
    return nullptr;
  }
  output->is_bigendian = false;
  output->is_dense = input.is_dense;

  output->fields.resize(3);
  for (int i = 0; i < 3; ++i) {
    auto & field = output->fields[static_cast<size_t>(i)];
    field.name = kFieldNames[i];
    field.count = 1;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.offset = static_cast<uint32_t>(i * sizeof(float));
  }

  output->point_step = static_cast<uint32_t>(3 * sizeof(float));
  output->row_step = output->point_step * output->width;
  output->data.resize(static_cast<size_t>(output->row_step) * output->height);

  sensor_msgs::PointCloud2ConstIterator<float> ix(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(input, "z");

  sensor_msgs::PointCloud2Iterator<float> ox(*output, "x");
  sensor_msgs::PointCloud2Iterator<float> oy(*output, "y");
  sensor_msgs::PointCloud2Iterator<float> oz(*output, "z");

  for (; ox != ox.end(); ++ox, ++oy, ++oz, ++ix, ++iy, ++iz) {
    *ox = *ix;
    *oy = *iy;
    *oz = *iz;
  }

  return output;
}

sensor_msgs::msg::PointCloud2::SharedPtr apply_voxel_grid(
  const sensor_msgs::msg::PointCloud2 & normalized,
  double leaf_size,
  rclcpp::Logger logger)
{
  if (leaf_size <= 0.0) {
    return std::make_shared<sensor_msgs::msg::PointCloud2>(normalized);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_xyz->reserve(static_cast<size_t>(normalized.width) * normalized.height);

  sensor_msgs::PointCloud2ConstIterator<float> ix(normalized, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(normalized, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(normalized, "z");

 for (; ix != ix.end(); ++ix, ++iy, ++iz) {
   pcl::PointXYZ p;
   p.x = *ix;
   p.y = *iy;
   p.z = *iz;
    cloud_xyz->push_back(p);
  }

  if (cloud_xyz->empty()) {
    return std::make_shared<sensor_msgs::msg::PointCloud2>(normalized);
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_xyz;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
  voxel_filter.setInputCloud(cloud_xyz);
  voxel_filter.filter(filtered_xyz);

  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(filtered_xyz, filtered_msg);
  filtered_msg.header = normalized.header;
  filtered_msg.is_dense = normalized.is_dense;

  // Ensure normalized layout again
  auto renormalized = normalize_cloud(filtered_msg, logger);
  if (!renormalized) {
    return std::make_shared<sensor_msgs::msg::PointCloud2>(normalized);
  }
  return renormalized;
}

}  // namespace

class CloudRepacker : public rclcpp::Node
{
public:
  CloudRepacker()
  : rclcpp::Node("cloud_repacker")
  {
    input_topic_ = declare_parameter<std::string>("input", "/SampleDrone/os_sensor/point_cloud");
    output_topic_ = declare_parameter<std::string>("output", "/scan_cloud");
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.0);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CloudRepacker::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "cloud_repacker: %s -> %s (voxel_leaf_size=%.3f)",
      input_topic_.c_str(), output_topic_.c_str(), voxel_leaf_size_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    auto normalized = normalize_cloud(*msg, get_logger());
    if (!normalized) {
      return;
    }

    auto output = apply_voxel_grid(*normalized, voxel_leaf_size_, get_logger());
    pub_->publish(*output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double voxel_leaf_size_ {0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace slam_stack

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_stack::CloudRepacker>());
  rclcpp::shutdown();
  return 0;
}
