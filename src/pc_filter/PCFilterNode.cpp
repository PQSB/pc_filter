#include "pc_filter/PCFilterNode.hpp"

#include "pc_filter/pc_filter_lib.hpp"

namespace pc_filter
{
PCFilterNode::PCFilterNode()
    : Node("pc_filter_node")
{
  filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", 10);

  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", 10, std::bind(&PCFilterNode::filter_cloud_callback, this, std::placeholders::_1));
}

void PCFilterNode::filter_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::high_resolution_clock::now();

  float min = -1.6f;
  float max = 1.6f;

  sensor_msgs::msg::PointCloud2 filtered_msg;
  filter_cloud_fov_2d(msg, filtered_msg, min, max);

  filtered_pc_pub_->publish(filtered_msg);
  // RCLCPP_INFO(this->get_logger(), "Publishing filtered point cloud");

  auto t1 = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  RCLCPP_INFO(this->get_logger(), "Callback time: %.3f ms", ms);
}

}  // namespace pc_filter
