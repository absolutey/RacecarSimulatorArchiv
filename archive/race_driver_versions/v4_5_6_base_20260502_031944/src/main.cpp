#include "rclcpp/rclcpp.hpp"
#include "race_driver/race_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<race_driver::RaceDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
