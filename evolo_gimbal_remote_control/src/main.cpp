#include "evolo_gimbal_remote_control/json_subscriber.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<evolo::JsonSubscriber>());
  rclcpp::shutdown();
  return 0;
}
