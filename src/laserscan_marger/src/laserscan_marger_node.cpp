#include "laserscan_marger/laserscan_marger.hpp"
#include <memory>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<laserscan_marger::LaserScanMarger>("laserscan_marger_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
