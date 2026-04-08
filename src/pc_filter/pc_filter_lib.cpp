#include "pc_filter/pc_filter_lib.hpp"

#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

void pc_filter::filter_cloud_x(const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2 &filtered_msg)
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

void pc_filter::filter_cloud_fov_2d(const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2 &filtered_msg)
{
  std_msgs::msg::Header original_header = msg->header;

  // Convertir la nube de puntos a PCL
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  pcl::fromROSMsg(*msg, *cloud);
  // Filtrar la nube de puntos
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point : *cloud)
  {

    float angle = std::atan2(point.y, point.x);

    // Frontal FOV de 180°: [-90°, +90°]
    if (angle > -M_PI/2 && angle < M_PI/2)
    {
      filtered_cloud->points.push_back(point);
    }
  }

  // Convertir la nube de puntos filtrada a sensor_msgs/PointCloud2
  pcl::toROSMsg(*filtered_cloud, filtered_msg);

  filtered_msg.header = original_header;
}
