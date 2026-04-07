#ifndef PC_FILTER__PC_FILTER_LIB_HPP_
#define PC_FILTER__PC_FILTER_LIB_HPP_

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pc_filter
{

void filter_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 &filtered_msg);

}
#endif  // PC_FILTER__PC_FILTER_LIB_HPP_
