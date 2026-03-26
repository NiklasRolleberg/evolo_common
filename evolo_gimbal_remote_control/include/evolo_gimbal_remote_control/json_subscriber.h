/* Class to handle a JSON string coming from a std_msgs::String topic.
 *
 * author: aldo teran
 * email: aldot@kth.se
 * license: MIT
 */

#ifndef __JSON_SUBSCRIBER__
#define __JSON_SUBSCRIBER__
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "z1_pro_msgs/msg/cam_cmd.hpp"

namespace evolo {

class JsonSubscriber : public rclcpp::Node {
 public:
  JsonSubscriber();

 private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Publisher<z1_pro_msgs::msg::CamCmd>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace evolo

#endif
