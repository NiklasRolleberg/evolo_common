/**
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#include <chrono>
#include <iostream>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Quaternion.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"

#include "GeographicLib/UTMUPS.hpp"

using namespace std::chrono_literals;

class SbgToOdom : public rclcpp::Node {
 public:
  SbgToOdom() : Node("sbg_nav_to_lolo_odom") {
    w_target_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "lolo/proxops/target_odom", 10);

    sbg_nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
        "/sbg/ekf_nav", 10,
        std::bind(&SbgToOdom::SbgNavCallback, this, std::placeholders::_1));

    sbg_quat_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfQuat>(
        "/sbg/ekf_quat", 10,
        std::bind(&SbgToOdom::SbgQuatCallback, this, std::placeholders::_1));

    // Timer for checking for and publishing updates..
    target_timer_ = this->create_wall_timer(
        100ms, std::bind(&SbgToOdom::target_timer_callback, this));

    // tf listener for utm->lolo/odom offset.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

 public:
  double x_;         // X position in UTM.
  double y_;         // Y position in UTM.
  double x_utm_offset_;  // X offset from utm->lolo/odom.
  double y_utm_offset_;  // Y offset from utm->lolo/odom.
  bool utm_init_ = false;

  double x_vel;  // SBG's X velocity.
  double y_vel;  // SBG's Y velocity.

  // Quat for odometry message.
  geometry_msgs::msg::Quaternion quat_msg;
  geometry_msgs::msg::Point pos_msg;
  std_msgs::msg::Header header_msg;


 private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // TODO: NED->ENU rotation.
  const gtsam::Rot3 ned_to_enu_ = gtsam::Rot3::RzRyRx(-M_PI / 2.0, 0.0, M_PI);


  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr w_target_pub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr sbg_nav_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr sbg_quat_sub_;

  rclcpp::TimerBase::SharedPtr target_timer_;

  // -----------------------------------------------------------------------
  void SbgNavCallback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
    if (!utm_init_) {
      return;
    }

    header_msg = msg->header;

    // Get lat/lon in UTM
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp,
                                    x_, y_);
    std::cout << "Position in UTM " << x_ << " " << y_ << "\n";
    std::cout << "UTM offset " << x_utm_offset_ << " " << y_utm_offset_ << "\n";

    x_ -= x_utm_offset_;
    y_ -= y_utm_offset_;

    pos_msg.x = x_;
    pos_msg.y = y_;
    pos_msg.z = 0.0;
  }

  // -----------------------------------------------------------------------
  void SbgQuatCallback(const sbg_driver::msg::SbgEkfQuat::SharedPtr msg) {
    double qx = msg->quaternion.x;
    double qy = msg->quaternion.y;
    double qz = msg->quaternion.z;
    double qw = msg->quaternion.w;
    const gtsam::Rot3 ned_to_sbg = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
    const gtsam::Rot3 enu_to_sbg = ned_to_enu_.inverse() * ned_to_sbg;
    const gtsam::Quaternion quat = enu_to_sbg.toQuaternion();

    this->quat_msg.x = quat.x();
    this->quat_msg.y = quat.y();
    this->quat_msg.z = quat.z();
    this->quat_msg.w = quat.w();
  }

  // -----------------------------------------------------------------------

  void getUtmOffset() {
    geometry_msgs::msg::TransformStamped utm_to_lolo_odom;
    try {
      utm_to_lolo_odom =
          tf_buffer_->lookupTransform("lolo/odom", "utm", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  "utm", "lolo/odom", ex.what());
      return;
    }
    x_utm_offset_ = utm_to_lolo_odom.transform.translation.x;
    y_utm_offset_ = utm_to_lolo_odom.transform.translation.y;
    utm_init_ = true;
  }

  // -----------------------------------------------------------------------
  nav_msgs::msg::Odometry OdomToMessage() {
    nav_msgs::msg::Odometry msg;
    msg.header = header_msg;
    msg.header.frame_id = "/lolo/odom";
    msg.child_frame_id = "target/base_link";
    msg.pose.pose.position = pos_msg;
    msg.pose.pose.orientation = quat_msg;
    return msg;
  }

  // -----------------------------------------------------------------------
  void target_timer_callback() {
    if (!utm_init_) {
      getUtmOffset();
    }
    w_target_pub_->publish(OdomToMessage());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::cout << "Starting fake target publisher.\n";
  rclcpp::spin(std::make_shared<SbgToOdom>());

  rclcpp::shutdown();

  return 0;
}
