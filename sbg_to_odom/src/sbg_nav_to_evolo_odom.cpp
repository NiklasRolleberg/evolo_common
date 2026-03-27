/**
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#include <chrono>
#include <iostream>

//#include <gtsam/geometry/Rot3.h>
//#include <gtsam/geometry/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geographic_msgs/msg/geo_point.hpp"

#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"

#include "GeographicLib/UTMUPS.hpp"

using namespace std::chrono_literals;

class SbgToOdom : public rclcpp::Node {
 public:
  SbgToOdom() : Node("sbg_nav_to_odom") {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "smarc/odom", 10);
    
    latlon_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoint>(
        "smarc/latlon", 10);

    sbg_nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
        "sbg/ekf_nav", 10,
        std::bind(&SbgToOdom::SbgNavCallback, this, std::placeholders::_1));

    sbg_quat_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfQuat>(
        "sbg/ekf_quat", 10,
        std::bind(&SbgToOdom::SbgQuatCallback, this, std::placeholders::_1));

    // Timer for checking for and publishing updates..
    target_timer_ = this->create_wall_timer(
        100ms, std::bind(&SbgToOdom::odom_timer_callback, this));

    // tf listener for utm->evolo/odom offset.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

 public:
  double x_;         // X position in UTM.
  double y_;         // Y position in UTM.
  double x_utm_offset_;  // X offset from utm->evolo/odom.
  double y_utm_offset_;  // Y offset from utm->evolo/odom.
  bool utm_init_ = false;

  double x_vel;  // SBG's X velocity.
  double y_vel;  // SBG's Y velocity.

  double latitude_;
  double longitude_;
  double altitude_;

  // Quat for odometry message.
  geometry_msgs::msg::Quaternion quat_msg;
  geometry_msgs::msg::Point pos_msg;
  std_msgs::msg::Header header_msg;


 private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Eigen::Matrix3d R_NED_to_ENU = (Eigen::Matrix3d() << 0, 1, 0,
                                                        1, 0, 0,
                                                        0, 0, -1).finished();
  Eigen::Matrix3d R_SBG_to_ROS =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr latlon_pub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfNav>::SharedPtr sbg_nav_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgEkfQuat>::SharedPtr sbg_quat_sub_;

  rclcpp::TimerBase::SharedPtr target_timer_;

  // -----------------------------------------------------------------------
  void SbgNavCallback(const sbg_driver::msg::SbgEkfNav::SharedPtr msg) {
    if (!utm_init_) {
      return;
    }

    header_msg = msg->header;

    latitude_ = msg->latitude;
    longitude_ = msg->longitude;
    altitude_ = 0.0 /*msg->altitude*/;

    // Get lat/lon in UTM
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(msg->latitude, msg->longitude, zone, northp,
                                    x_, y_);
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
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond ned_to_sbg(qw, qx, qy, qz);
    ned_to_sbg.normalize();
    Eigen::Matrix3d R_sbg = ned_to_sbg.toRotationMatrix();

    Eigen::Matrix3d R_ros = R_NED_to_ENU * R_sbg * R_SBG_to_ROS;

    Eigen::Quaterniond q_ros(R_ros);
    q_ros.normalize();

    this->quat_msg.x = q_ros.x();
    this->quat_msg.y = q_ros.y();
    this->quat_msg.z = q_ros.z();
    this->quat_msg.w = q_ros.w();

    // TODO velocities
  }

  // -----------------------------------------------------------------------

  void getUtmOffset() {
    geometry_msgs::msg::TransformStamped utm_to_odom;
    try {
      utm_to_odom =
          tf_buffer_->lookupTransform("utm", "evolo/odom", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  "utm", "evolo/odom", ex.what());
      return;
    }
    x_utm_offset_ = utm_to_odom.transform.translation.x;
    y_utm_offset_ = utm_to_odom.transform.translation.y;
    utm_init_ = true;
  }

  // -----------------------------------------------------------------------
  nav_msgs::msg::Odometry OdomToMessage() {
    nav_msgs::msg::Odometry msg;
    msg.header = header_msg;
    msg.header.frame_id = "evolo/odom";
    msg.child_frame_id = "evolo/base_link";
    msg.pose.pose.position = pos_msg;
    msg.pose.pose.orientation = quat_msg;
    return msg;
  }

  // -----------------------------------------------------------------------
  geographic_msgs::msg::GeoPoint LatLonToMessage() {
    geographic_msgs::msg::GeoPoint msg;
    msg.latitude = latitude_;
    msg.longitude = longitude_;
    msg.altitude = altitude_;

    return msg;
  }

  // -----------------------------------------------------------------------
  void odom_timer_callback() {
    if (!utm_init_) {
      getUtmOffset();
    }
    odom_pub_->publish(OdomToMessage());

    latlon_pub_->publish(LatLonToMessage());

    //Broadcast base_link transform
    geometry_msgs::msg::TransformStamped t;
    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = header_msg.stamp;
    t.header.frame_id = "evolo/odom";
    t.child_frame_id = "evolo/base_link";

    // position coordinates
    t.transform.translation.x = pos_msg.x;
    t.transform.translation.y = pos_msg.y;
    t.transform.translation.z = pos_msg.z;

    //Orientation
    t.transform.rotation.x = quat_msg.x;
    t.transform.rotation.y = quat_msg.y;
    t.transform.rotation.z = quat_msg.z;
    t.transform.rotation.w = quat_msg.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::cout << "Starting evolo odom publisher.\n";
  rclcpp::spin(std::make_shared<SbgToOdom>());

  rclcpp::shutdown();

  return 0;
}
