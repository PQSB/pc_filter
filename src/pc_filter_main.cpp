#include <memory>

#include "pc_filter/PCFilterNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<pc_filter::PCFilterNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
