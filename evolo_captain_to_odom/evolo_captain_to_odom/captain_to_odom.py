#!/usr/bin/python

# General
import yaml
import math

# ROS
import rclpy
from rclpy.node import Node
# from message_filters import Subscriber, ApproximateTimeSynchronizer

# Transforms
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf_transformations

from tf2_geometry_msgs import do_transform_pose_stamped

# Messages
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion, PointStamped, Pose
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import Imu

# SMaRC Topics
from evolo_msgs.msg import Topics as evoloTopics
from evolo_msgs.msg import CaptainState
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_mission_msgs.msg import Topics as MissionTopics

# Geo transform imports
# Switching from Ozers service to pyproj or utm lib
# from geodesy.utm import UTMPoint
# Note: pyproj is a little fuller featured but requires setting up the transformer
# from pyproj import Proj, Transformer, CRS
import pyproj
import utm
import json

try:
    from .helpers.spatial_helpers import heading_to_yaw
    from .helpers.spatial_helpers import yaw_to_heading
    from .helpers.spatial_helpers import normalize_angle_rad
    from .helpers.spatial_helpers import normalize_angle_deg
except ImportError:
    from helpers.spatial_helpers import heading_to_yaw
    from helpers.spatial_helpers import yaw_to_heading
    from helpers.spatial_helpers import normalize_angle_rad
    from helpers.spatial_helpers import normalize_angle_deg


class Captain2Odom(Node):
    """
    This node will convert output from the INS to odometry messages
    """

    def __init__(self, namespace=None):
        super().__init__("ins_2_odom", namespace=namespace)
        self._log("Starting node defined in ins_2_odom_complete.py")

        # ===== Declare parameters =====
        # Default values set in declare_parameters()
        self.declare_node_parameters()

        # ===== Get parameters =====
        # Note: All parameters must be declared first! see self.declare_parameters
        # Example: self.map_frame = self.get_parameter("map_frame").value
        self.input_topic = self.get_parameter("input_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value

        # Data
        self.millis = None
        self.latitude = None
        self.longitude = None
        self.sog = None
        self.cog = None
        self.pitch = None
        self.roll = None

        # Coordinate transform related attributes
        self.utm_zone = None
        self.utm_band = None
        self.projection_transformer = None

        # tf transform related attributes
        self.utm_frame = None
        self.output_odom_frame = self.get_parameter("output_odom_frame").value
        self.odom_child_frame_id = self.get_parameter("base_link_frame").value  # Should this be hard coded? NO!

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Status
        # are any of these not redundent?!
        self.utm_zone_status = False  # status if we know what UTM zone and band we are working in
        self.utm_frame_status = False  # Status if utm frame has been set
        self.transform_status = False  # Status if we have a valid transform from 'utm' to 'map'
        self.projection_status = False  # Status if we have a valid set-up for out pyproj object

        # Parameters for controlling behavior
        self.correct_meridian_convergence = self.get_parameter("correct_meridian_convergence").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.publisher_period = 0.5  # Rate at which to publish converted
        self.output_rate = self.get_parameter("output_rate").value
        self.conversion_timeout = 2.5  # Timeout for tf related transforms

        self.status_last_time = None
        self.status_min_period = 2.0

        self.verbose_setup = self.get_parameter("verbose_setup").value
        self.verbose_conversion = self.get_parameter("verbose_conversion").value

        self.log_log = None

        # Subscribers
        # IND data subscription (ins_sub):

        # TF Messages subscription (frame_sub):
        # This sub is used to determine the utm zone that we are working in.
        # Once determined we will remove this sub

        # Original un synced
        self._log(f"Subscribing to INS topic :{self.input_topic}")
        self.ins_sub = self.create_subscription(msg_type=CaptainState, topic=self.input_topic,
                                                callback=self.captain_callback,
                                                qos_profile=10)

        # self.ins_sub = Subscriber(self, Ins, self.input_topic)  # msg_type and topic name
        # self.imu_sub = Subscriber(self, Imu, self.input_imu_topic)
        #
        # # synchronizer
        # queue_size = 10
        # max_delay = 1
        # self.time_sync = ApproximateTimeSynchronizer([self.ins_sub, self.imu_sub],
        #                                              queue_size, max_delay)
        # self.time_sync.registerCallback(self.sync_callback)

        # SMARC topics publishers.
        self.odom_pub = self.create_publisher(msg_type=Odometry, topic=self.output_odom_topic,
                                              qos_profile=10)
        self.lat_lon_pub = self.create_publisher(msg_type=GeoPoint,
                                                 topic=SmarcTopics.POS_LATLON_TOPIC,
                                                 qos_profile=10)
        self.depth_pub = self.create_publisher(msg_type=Float32,
                                               topic=SmarcTopics.DEPTH_TOPIC,
                                               qos_profile=10)
        self.heading_pub = self.create_publisher(msg_type=Float32,
                                                 topic=SmarcTopics.HEADING_TOPIC,
                                                 qos_profile=10)
        self.course_pub = self.create_publisher(msg_type=Float32,
                                                topic=SmarcTopics.COURSE_TOPIC,
                                                qos_profile=10)
        self.speed_pub = self.create_publisher(msg_type=Float32,
                                               topic=SmarcTopics.SPEED_TOPIC,
                                               qos_profile=10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        # Odom publisher timer (publisher_timer):
        # Publishing is handled in a timer (publish_timer) and its callback (publisher_callback)
        # The rate is set by self.publish_period.

        self._log(f"Output rate: {self.output_rate}")
        self.publisher_timer = self.create_timer(timer_period_sec=(1.0/self.output_rate),
                                                 callback=self.publisher_callback)

    def _log(self, message):
        self.get_logger().info(message)

    def _log_meter(self, message):
        if message == self.log_log:
            return

        self.log_log = message
        self.get_logger().info(message)

    # Basic node set up
    def declare_node_parameters(self):
        # Declare all the default values for parameters

        # Topic names
        # Subscriptions
        self.declare_parameter("input_topic", evoloTopics.EVOLO_CAPTAIN_STATE)

        # Publishers
        self.declare_parameter("output_odom_topic", SmarcTopics.ODOM_TOPIC)

        # Frames
        self.declare_parameter("output_odom_frame", "evolo/odom")
        self.declare_parameter("base_link_frame", "evolo/base_link")

        # Behavior
        self.declare_parameter("correct_meridian_convergence", True)
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("output_rate", 20.0)

        # Verbose output
        self.declare_parameter("verbose_setup", True)
        self.declare_parameter("verbose_conversion", True)

    # Projection transform functions
    def determine_utm_zone_tf_names(self):
        if self.utm_zone_status:
            return

        if self.verbose_setup:
            self._log("Determining UTM zone and band")

        # yaml method for finding the utm zone
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames_dict = yaml.safe_load(frames_yaml)
        if len(frames_dict) < 1:
            if self.verbose_setup:
                self._log("TF Buffer has no frames")
            return
        frame_names = frames_dict.keys()
        if self.verbose_setup:
            print(f"Frame names: {frame_names}")

        # Verify  that utm frame is present
        if "utm" not in frame_names:
            return

        # Zone info is contained in the name of the utm frame parent, utm_%%_##
        utm_parent = frames_dict["utm"]["parent"]
        utm_parent_split = utm_parent.split("_")

        # Check that split results in expected number of elements, 3
        if len(utm_parent_split) != 3:
            return

        self.utm_zone = int(utm_parent_split[1])  # [[utm_parent_split[1], utm_parent_split[2]]
        self.utm_band = utm_parent_split[2]
        self.utm_frame = utm_parent
        self.utm_zone_status = True
        self.utm_frame_status = True

        self.set_up_projection()

        self._log(f'UTM Zone: {self.utm_zone}')

    def utm_to_frame_status(self, frame_name):
        """
        Given that the utm frame name is defined,
        will check if there is a possible transform between utm frame and input frame
        """
        if self.utm_frame is None:
            return False

        try:
            transform_status = self.tf_buffer.can_transform(target_frame=self.utm_frame,
                                                            source_frame=self.output_odom_frame,
                                                            time=rclpy.time.Time(),
                                                            return_debug_tuple=True)

            if not transform_status:
                self._log("Invalid transform")

            return transform_status

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Error while checking transform: {e}')
            return False

    # Callbacks
    # ins_callback: save incoming ins message to self.current_ins
    # frame_callback: Checks frame names to find the "utm_{zone}_band}" -> sets self.utm_zone_status
    # publisher_callback: Converts the most recent ins message and converts it appropriately
    # tf_name_callback: (Alternative) Checks the tf buffer for the correct naming convention to determine the root
    # of the tf tree

    def captain_callback(self, msg: CaptainState):
        #self.get_logger().info("Received ROS message " + str(msg.data))
        self.millis = msg.ms
        self.latitude = msg.lat
        self.longitude = msg.lon
        self.sog = msg.sog
        self.cog = msg.cog
        self.pitch = msg.pitch
        self.roll = msg.roll
        
        self.get_logger().info("millis " + str(self.millis))
        self.get_logger().info("latitide " + str(self.latitude))
        self.get_logger().info("longitude " + str(self.longitude))
        self.get_logger().info("sog " + str(self.sog))
        self.get_logger().info("cog " + str(self.cog))
        self.get_logger().info("pitch " + str(self.pitch))
        self.get_logger().info("roll " + str(self.roll))

    def publisher_callback(self):

        self.current_2_odom()

    def compute_course(self, yaw_enu, vel_x, vel_y):
        """
        Compute the course of Lolo using her current heading
        and the velocity vector in the plane.

        Returns course angle in NED.
        """
        cross_track_yaw = math.atan2(vel_y, vel_x)
        course_yaw = normalize_angle_rad(yaw_enu + cross_track_yaw)
        return yaw_to_heading(course_yaw)

    def current_2_odom(self):

        # Check that ins and imu data has been recieved
        if self.latitude is None or self.longitude is None:
            if self.verbose_setup:
                self._log("No data yet")
            return

        self.determine_utm_zone_tf_names()

        if self.utm_frame is None:
            if self.verbose_setup:
                self._log("UTM frame not initialized")
            return

        if self.projection_transformer is None:
            if self.verbose_setup:
                self._log("Projection not initialized")
            return

        # Extract information

        lat = self.latitude
        lon = self.longitude
        altitude = 0.0

        roll = self.roll
        pitch = self.pitch
        heading = self.cog

        # Convert Lat and Lon to UTM
        easting, northing = self.projection_transformer.transform(lon, lat)
        factors = self.projection_transformer.get_factors(lon, lat)

        if self.correct_meridian_convergence:
            meridian_convergence = factors.meridian_convergence
            heading = (heading - meridian_convergence) % 360

        # Convert thr INS roll, pitch, yaw to a quaternion
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_enu = heading_to_yaw(heading)

        xyz_vehicle_frame_velocities = self.sog

        # GeoPoint for LatLon topic.
        geopoint = GeoPoint()
        geopoint.latitude = lat
        geopoint.longitude = lon
        geopoint.altitude = altitude

        # Heading and course for topics.
        course = Float32()
        course.data = self.cog

        heading_ned = Float32()
        heading_ned.data = heading

        # Velocity in the plane.
        speed = Float32()
        speed.data = self.sog

        # Depth.
        depth = Float32()
        depth.data = -altitude

        # No longer checking for utm zon matches, the initial zone will be use for the duration of the mission
        # Check if the utm zone from utm lib matches the utm zone from tf frame names
        # if zone != self.utm_zone:
        #     self._log(f"utm zone mismatch between lat/lon zone ({zone}) and tf root zone ({self.utm_zone})")
        #     return

        # At this point we have 'valid' utm coords, utm frame name, but we still need to check for a valid transform
        # Between the utm frame (map) and the desired frame(odom)

        pose_utm = PoseStamped()
        pose_utm.header.frame_id = self.utm_frame
        pose_utm.header.stamp = self.get_clock().now().to_msg()

        pose_utm.pose.position.x = easting
        pose_utm.pose.position.y = northing
        pose_utm.pose.position.z = altitude

        pose_quaternion_values = tf_transformations.quaternion_from_euler(roll_rad,
                                                                          pitch_rad,
                                                                          yaw_enu)
        pose_utm.pose.orientation.x = pose_quaternion_values[0]
        pose_utm.pose.orientation.y = pose_quaternion_values[1]
        pose_utm.pose.orientation.z = pose_quaternion_values[2]
        pose_utm.pose.orientation.w = pose_quaternion_values[3]

        # Perform transformation to the correct frame
        try:
            transform = self.tf_buffer.lookup_transform(target_frame=self.output_odom_frame,
                                                        source_frame=self.utm_frame,
                                                        time=rclpy.time.Time(),
                                                        timeout=rclpy.time.Duration(seconds=self.conversion_timeout))

            pose_odom = do_transform_pose_stamped(pose=pose_utm,
                                                  transform=transform)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.verbose_conversion:
                self.get_logger().warn(
                    f"TF transform from {self.utm_frame} to {self.output_odom_frame} not available: {e}")
            return

        # Publish odometry
        current_stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = current_stamp
        odom.header.frame_id = self.output_odom_frame
        odom.child_frame_id = self.odom_child_frame_id
        odom.pose.pose = pose_odom.pose

        # Rates
        odom.twist.twist.linear.x = self.sog

        # Publish messages.

        self.lat_lon_pub.publish(geopoint)
        self.speed_pub.publish(speed)
        self.heading_pub.publish(heading_ned)
        self.course_pub.publish(course)
        self.depth_pub.publish(depth)
        self.odom_pub.publish(odom)

        if self.publish_tf:
            # Publish TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_stamp
            tf_msg.header.frame_id = self.output_odom_frame
            tf_msg.child_frame_id = self.odom_child_frame_id
            tf_msg.transform.translation.x = pose_odom.pose.position.x
            tf_msg.transform.translation.y = pose_odom.pose.position.y
            tf_msg.transform.translation.z = pose_odom.pose.position.z
            tf_msg.transform.rotation = pose_odom.pose.orientation

            self.tf_broadcaster.sendTransform(tf_msg)

    def valid_output_time(self):

        if not self.limit_output_rate:
            # self._log("Time_check: no limit")
            return True

        if self.last_odom_publish is None:
            # self._log("Time_check: First time")
            return True

        now = self.get_clock().now().nanoseconds * 1e-9

        if (now - self.last_odom_publish) < (1.0 / self.output_rate):
            # self._log("Time_check: Invalid time")
            return False

        # self._log(f"Time_check: Valid time {now - self.last_odom_publish} / {1.0/self.output_rate} ")
        return True

    def check_status_valid(self, current_time: float):
        """
        check if it is time to provide a status update
        """
        if self.status_last_time is None:
            self.status_last_time = current_time
            status_valid = True
        elif current_time - self.status_last_time >= self.status_min_period:
            self.status_last_time = current_time
            status_valid = True
        else:
            status_valid = False

        return status_valid

    # Setup
    def set_utm_zone_automatic(self, latitude, longitude):
        utm_zone = utm.latlon_to_zone_number(latitude=latitude, longitude=longitude)

        if 1 <= utm_zone <= 60:
            self.utm_zone = utm_zone
            self.utm_zone_status = True

    def set_up_projection(self):
        if not self.utm_zone_status:
            return False

        if self.projection_transformer is not None:
            return True

        if self.verbose_setup:
            self._log(f'Setting up projection for zone {self.utm_zone}')

        # Create a transformer
        self.projection_transformer = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')


def main(args=None, namespace=None):
    rclpy.init(args=args)
    node = Captain2Odom(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    default_namespace = "evolo"
    main(namespace=default_namespace)
