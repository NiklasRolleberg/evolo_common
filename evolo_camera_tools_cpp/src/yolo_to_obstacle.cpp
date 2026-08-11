#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "smarc_msgs/msg/topics.hpp"
#include "evolo_msgs/msg/topics.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "yolo_msgs/msg/detection.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

//using namespace std::chrono_literals;



class YoloDetectionToOdom : public rclcpp::Node
{
public:
  YoloDetectionToOdom() : Node("yolo_detection_to_odom")
  {

    // Declare parameters with default values
    this->declare_parameter<bool>("publish_viz", true);
    this->declare_parameter<float>("camera_aperture", 50.0);
    this->declare_parameter<float>("projection_distance", 25);

    publish_viz            = this->get_parameter("publish_viz").as_bool();
    camera_aperture        = this->get_parameter("camera_aperture").as_double();
    projection_distance    = this->get_parameter("projection_distance").as_double();


    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  publish_viz = %d", publish_viz);
    RCLCPP_INFO(this->get_logger(), "  camera_aperture = %f", camera_aperture);
    RCLCPP_INFO(this->get_logger(), "  projection_distance = %f", projection_distance);

    //tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/evolo/rviz/camera_obstacle", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/evolo/obstacles", 10);

    detection_subscription_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
      "/yolo/tracking", 10,
      std::bind(&YoloDetectionToOdom::detection_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node started");
  }

  void detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    for (int i=0;i<msg->detections.size();i++) {
      if(msg->detections[i].score < detection_threshold) continue;

      try {
        int image_width = msg->detections[i].mask.width;
        int image_height = msg->detections[i].mask.height;
        int pixel_x = msg->detections[i].bbox.center.position.x - 0.5*image_width;
        int pixel_y = msg->detections[i].bbox.center.position.y - 0.5*image_height;
        float angle_per_pixel = (camera_aperture * M_PI / 180.0) / image_width;

        float roll = 0;
        float yaw = -1.0 * pixel_x * angle_per_pixel;
        float pitch = 1.0 * pixel_y * angle_per_pixel;

        tf2::Quaternion q_camera;
        q_camera.setRPY(roll, pitch, yaw);

        // Rotate to base_link
        // Transform from camera_link -> evolo/base_link
        geometry_msgs::msg::TransformStamped tf =
            tf_buffer_->lookupTransform(
                "evolo/base_link",
                "evolo/z1_camera_link",
                tf2::TimePointZero);

        // Convert the TF transform to a tf2 Transform
        tf2::Transform T;
        tf2::fromMsg(tf.transform, T);

        // Transform the orientation
        tf2::Quaternion q_base = T.getRotation() * q_camera;
        q_base.normalize();

        //Move point from (0,0) to q_base*projectiondistance
        // 10 meters in the +X direction of q_base
        tf2::Vector3 direction(1.0, 0.0, 0.0);
        direction = tf2::quatRotate(q_base, direction);
        direction *= projection_distance;

        geometry_msgs::msg::PointStamped detection_position_base_link;
        detection_position_base_link.header.frame_id = "evolo/base_link";
        detection_position_base_link.header.stamp = this->get_clock()->now();
        detection_position_base_link.point.x += direction.x();
        detection_position_base_link.point.y += direction.y();
        detection_position_base_link.point.z += direction.z();

        //Transform point to global frame
  
        geometry_msgs::msg::PointStamped detection_global_frame = tf_buffer_->transform(
          detection_position_base_link,
          "evolo/odom",
          tf2::durationFromSec(1.0));
        
        
        //Publish Odom message
        nav_msgs::msg::Odometry odom_obstacle;
        odom_obstacle.header = detection_global_frame.header;
        odom_obstacle.child_frame_id = "z1_obstacle_" + i;
        odom_obstacle.pose.pose.position = detection_position_base_link.point;
        odom_publisher_->publish(odom_obstacle);


        //Add marker array
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "evolo/odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "camera_detections";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = detection_global_frame.point.x;
        marker.pose.position.y = detection_global_frame.point.y;
        marker.pose.position.z = detection_global_frame.point.z;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 10.0;
        marker.scale.y = 10.0;
        marker.scale.z = 10.0;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration::from_seconds(10.0);

        marker_array.markers.push_back(marker); 
      }
      catch (const tf2::TransformException & ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      }
    }

    //Publish marker array
    if(marker_array.markers.size() > 0) {
      marker_publisher_->publish(marker_array);
      RCLCPP_INFO(this->get_logger(), "Published detections: %ld", msg->detections.size());
    }
  }

private:
  
  //Settings
  bool publish_viz = true;
  float camera_aperture = 50;
  float projection_distance = 25;
  float detection_threshold = 0.6;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloDetectionToOdom>());
  rclcpp::shutdown();
  return 0;
}