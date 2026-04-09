#ifndef PC_FILTER__PC_FILTER_LIB_HPP_
#define PC_FILTER__PC_FILTER_LIB_HPP_

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cmath>

namespace pc_filter
{

void filter_cloud_x(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg);

void filter_cloud_fov_2d(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg,
    float min_angle = -M_PI/2.0f, float max_angle = M_PI/2.0f);

}
#endif  // PC_FILTER__PC_FILTER_LIB_HPP_
