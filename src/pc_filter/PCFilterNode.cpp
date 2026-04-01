#include "pc_filter/PCFilterNode.hpp"
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pc_filter
{
PCFilterNode::PCFilterNode()
    : Node("pc_filter_node")
{
  // Crear el publicador
  filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", 10);

  // Crear el suscriptor
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", 10, std::bind(&PCFilterNode::filter_cloud_callback, this, std::placeholders::_1));
}

void PCFilterNode::filter_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2 &filtered_msg)
{
  std_msgs::msg::Header original_header = msg->header;

  // Convertir la nube de puntos a PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  // Convertir de PCLPointCloud2 a PointCloud<pcl::PointXYZ>
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  // Filtrar la nube de puntos
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point : *cloud)
  {
    if (point.x >= 0.0)
    {
      filtered_cloud->points.push_back(point);
    }
  }
  // Convertir la nube de puntos filtrada a PCLPointCloud2
  pcl::PCLPointCloud2 pcl_filtered_pc2;
  pcl::toPCLPointCloud2(*filtered_cloud, pcl_filtered_pc2);
  // Convertir de PCLPointCloud2 a sensor_msgs/PointCloud2
  pcl_conversions::fromPCL(pcl_filtered_pc2, filtered_msg);

  filtered_msg.header = original_header;
}

void PCFilterNode::filter_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 filtered_msg;
  filter_cloud(msg, filtered_msg);

  // Publicar la nube de puntos filtrada
  filtered_pc_pub_->publish(filtered_msg);
  std::cerr << "\nPublishing filtered point cloud\n";
}

}  // namespace pc_filter
