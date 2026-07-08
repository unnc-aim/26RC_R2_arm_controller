#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arm_controller/arm_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_controller::ArmControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
