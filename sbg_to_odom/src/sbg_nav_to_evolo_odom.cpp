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

#include "sbg_driver/msg/sbg_ekf_quat.hpp"
#include "sbg_driver/msg/sbg_ekf_nav.hpp"

#include "GeographicLib/UTMUPS.hpp"

using namespace std::chrono_literals;

class SbgToOdom : public rclcpp::Node {
 public:
  SbgToOdom() : Node("sbg_nav_to_odom") {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "evolo/smarc/odom", 10);

    sbg_nav_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfNav>(
        "/sbg/ekf_nav", 10,
        std::bind(&SbgToOdom::SbgNavCallback, this, std::placeholders::_1));

    sbg_quat_sub_ = this->create_subscription<sbg_driver::msg::SbgEkfQuat>(
        "/sbg/ekf_quat", 10,
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

  // Quat for odometry message.
  geometry_msgs::msg::Quaternion quat_msg;
  geometry_msgs::msg::Point pos_msg;
  std_msgs::msg::Header header_msg;


 private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // TODO: NED->ENU rotation.
    //const gtsam::Rot3 ned_to_enu_ = gtsam::Rot3::RzRyRx(-M_PI / 2.0, 0.0, M_PI);
    Eigen::Matrix3d R_NED_to_ENU = (Eigen::Matrix3d() << 0, 1, 0,
                                                         1, 0, 0,
                                                         0, 0, -1).finished();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
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

    //std::cout << "SBG quat received "<< std::endl;
    
    // TODO: NED->ENU rotation.
    //const gtsam::Rot3 ned_to_enu_ = gtsam::Rot3::RzRyRx(-M_PI / 2.0, 0.0, M_PI);


    // Convert quaternion to rotation matrix
    Eigen::Quaterniond ned_to_sbg(qw, qx, qy, qz);
    Eigen::Matrix3d rotationMatrix = ned_to_sbg.toRotationMatrix();

    // Apply the transformation matrix to the rotation matrix
    Eigen::Matrix3d newRotationMatrix = R_NED_to_ENU.inverse() * rotationMatrix;// * R_NED_to_ENU.transpose();

    // Convert back to a quaternion in the ENU frame
    Eigen::Quaterniond q_ENU(newRotationMatrix);

    //for reference
    //const gtsam::Rot3 ned_to_sbg = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
    //const gtsam::Rot3 enu_to_sbg = ned_to_enu_.inverse() * ned_to_sbg;
    //const gtsam::Quaternion quat = enu_to_sbg.toQuaternion();
    //this->quat_msg.x = enu_to_sbg.x();
    //this->quat_msg.y = enu_to_sbg.y();
    //this->quat_msg.z = enu_to_sbg.z();
    //this->quat_msg.w = enu_to_sbg.w();

    //this->quat_msg.x = q_ENU.x();
    //this->quat_msg.y = q_ENU.y();
    //this->quat_msg.z = q_ENU.z();
    //this->quat_msg.w = q_ENU.w();

    this->quat_msg.x= msg->quaternion.x;
    this->quat_msg.y= msg->quaternion.y;
    this->quat_msg.z= msg->quaternion.z;
    this->quat_msg.w= msg->quaternion.w;

    //TODO velocities


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
    msg.header.frame_id = "/evolo/odom";
    msg.child_frame_id = "evolo/base_link";
    msg.pose.pose.position = pos_msg;
    msg.pose.pose.orientation = quat_msg;
    return msg;
  }

  // -----------------------------------------------------------------------
  void odom_timer_callback() {
    if (!utm_init_) {
      getUtmOffset();
    }
    odom_pub_->publish(OdomToMessage());

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

  std::cout << "Starting fake target publisher.\n";
  rclcpp::spin(std::make_shared<SbgToOdom>());

  rclcpp::shutdown();

  return 0;
}
