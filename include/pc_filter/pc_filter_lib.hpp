#ifndef PC_FILTER__PC_FILTER_LIB_HPP_
#define PC_FILTER__PC_FILTER_LIB_HPP_

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct Detection {
    std::string category;
    float x, y, z;
    float w, l, h;
    float ry;
    float score;
};

namespace pc_filter
{

void filter_cloud_x(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg);

void filter_cloud_fov_2d(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg,
    float min_angle = -M_PI/2.0f, float max_angle = M_PI/2.0f);

int
filter_detections_from_cloud(
    const std::shared_ptr<std::vector<Detection>>& dets,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const bool from_camera);

}

#endif  // PC_FILTER__PC_FILTER_LIB_HPP_
