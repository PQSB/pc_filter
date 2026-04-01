#ifndef PC_FILTER_PCFILTERNODE_HPP
#define PC_FILTER_PCFILTERNODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pc_filter
{
class PCFilterNode : public rclcpp::Node
{
public:
  PCFilterNode();

private:
  void filter_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

  void filter_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2 &filtered_msg);
};

}  // namespace pc_filter

#endif  // PC_FILTER_PCFILTERNODE_HPP
