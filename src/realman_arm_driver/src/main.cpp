#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "realman_arm_driver/realman_arm_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<realman_arm_driver::RealmanArmNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
