#include "evolo_gimbal_remote_control/json_subscriber.h"

namespace evolo {


JsonSubscriber::JsonSubscriber() : Node("gimbal_camera_remote") {
  // Declare parameters for topic names.
  this->declare_parameter<std::string>("string_input_topic",
                                       "mqtt/gimbal_command");
  this->declare_parameter<std::string>("camcmd_output_topic", "gimbal/cam_cmd");
  input_topic_ = this->get_parameter("string_input_topic").as_string();
  output_topic_ = this->get_parameter("camcmd_output_topic").as_string();

  // Create subscriber and publisher.
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      input_topic_, 10,
      std::bind(&JsonSubscriber::topic_callback, this, std::placeholders::_1));
  publisher_ =
      this->create_publisher<z1_pro_msgs::msg::CamCmd>(output_topic_, 10);
}

void JsonSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) {

  // Instantiate the CamCmd message and populate it with the JSON string.
  z1_pro_msgs::msg::CamCmd cam_msg;
  // FIXME: we'll hardcode the body frame for now until we figure out
  // how to use the xModes.
  cam_msg.frame = z1_pro_msgs::msg::CamCmd::BODY;

  // Encapsulate in catch in case of malformed JSON.
  try {
    nlohmann::json j = nlohmann::json::parse(msg->data);

    if (j.contains("roll") && j["roll"].is_number()) {
      cam_msg.roll = j["roll"];
    }
    if (j.contains("pitch") && j["pitch"].is_number()) {
      cam_msg.pitch = j["pitch"];
    }
    if (j.contains("yaw") && j["yaw"].is_number()) {
      cam_msg.yaw = j["yaw"];
    }

    // FIXME: How do we implement these in practice?
    if (j.contains("yawMode") && j["yawMode"].is_string()) {
      const std::string yawMode = j["yawMode"];
      RCLCPP_INFO(this->get_logger(), "yawMode: %s", yawMode.c_str());
    }
    if (j.contains("pitchMode") && j["pitchMode"].is_string()) {
      const std::string pitchMode = j["pitchMode"];
      RCLCPP_INFO(this->get_logger(), "pitchMode: %s", pitchMode.c_str());
    }
    if (j.contains("rollMode") && j["rollMode"].is_string()) {
      const std::string rollMode = j["rollMode"];
      RCLCPP_INFO(this->get_logger(), "rollMode: %s", rollMode.c_str());
    }

    if (j.contains("POI") && j["POI"].is_object()) {
      auto poi = j["POI"];

      if (poi.contains("lat") && poi["lat"].is_number() && poi.contains("lon") &&
          poi["lon"].is_number() && poi.contains("alt") && poi["alt"].is_number()) {
        cam_msg.poi.latitude = poi["lat"];
        cam_msg.poi.longitude = poi["lon"];
        cam_msg.poi.altitude = poi["alt"];
      }
    }

  } catch (const nlohmann::json::parse_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Raw message: %s", msg->data.c_str());
  } catch (const nlohmann::json::type_error& e) {
    RCLCPP_ERROR(this->get_logger(), "JSON type error: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
  }

  // Publish CamCmd msg if the above was successful.
  publisher_->publish(cam_msg);
}

}  // namespace evolo

