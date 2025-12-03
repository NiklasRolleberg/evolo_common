# General python imports
import utm

# ROS2 specific imports
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# ROS2 messages
from geometry_msgs.msg import TransformStamped
from evolo_msgs.msg import Topics as evoloTopics
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgEkfQuat
from std_msgs.msg import String
import json

#hard coded topics

class MapOdomInitializer(Node):
    """
    Initializes the TF tree and use the initial GPS position from the INS to define the "odom" frame.

    Static transforms are republished at the specified rate.

    base_transform: "utm_{zone}_{band}" -> "utm"
    odom_transform: "utm" -> "odom"

    Conventions used:
    "utm_{zone}_{band}" -> "utm" -> "odom"

    """

    def __init__(self, namespace=None):
        super().__init__('map_odom_initializer', namespace=namespace)
        
        self.declare_parameter('update_rate', 1.0)
        self.update_rate = self.get_parameter('update_rate').value

        self.declare_parameter("verbose", True)
        self.verbose = self.get_parameter("verbose").value

        self.declare_parameter("sbg_nav_topic", "/sbg/ekf_nav")
        captain_topic = self.get_parameter("sbg_nav_topic").value

        self._log(f'map -> odom initialization')
        self._log(f'Waiting for initial Lat/Lon coordinates: {captain_topic}')

        self.base_transform = None
        self.odom_transform = None

        self.subscription = self.create_subscription(
            msg_type=SbgEkfNav,
            topic=captain_topic,
            callback=self.sbg_callback,
            qos_profile=10
        )

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.origin_set = False

        # Timer for republishing static transforms, This can be done at a pretty slow rate
        # The purpose of republishing is to account for situations in which the ros bag is split
        self.create_timer(timer_period_sec=float(1.0 / self.update_rate), callback=self.transform_timer)

    def sbg_callback(self, msg: SbgEkfNav):
        if self.origin_set:
            return
        
        #TODO check if sbg pos is valid /aligned
        
        captain_latitude = msg.latitude
        captain_longitude = msg.longitude

        latitude = float(captain_latitude)
        longitude = float(captain_longitude)
        altitude = 0.0

        # Use the first GPS fix as map origin
        easting, northing, zone, letter = utm.from_latlon(latitude, longitude)

        # sets the transform so that the `odom` frame origin (0,0) is placed at the lat/lon-based UTM position in `map`.

        # Time used ofr stamping the TF messages
        now = self.get_clock().now().to_msg()

        # Define utm root (includes zone and band) to utm transform
        self.base_transform = TransformStamped()
        self.base_transform.header.stamp = now
        self.base_transform.header.frame_id = f'utm_{zone}_{letter}'
        self.base_transform.child_frame_id = 'utm'
        self.base_transform.transform.translation.x = 0.0
        self.base_transform.transform.translation.y = 0.0
        self.base_transform.transform.translation.z = 0.0
        self.base_transform.transform.rotation.w = 1.0  # Identity rotation

        # Define utm to odom transform, based on initial lat/lon from INS
        self.odom_transform = TransformStamped()
        self.odom_transform.header.stamp = now
        self.odom_transform.header.frame_id = f'utm'
        # TODO Check - What should the child frame be?
        self.odom_transform.child_frame_id = 'evolo/odom'
        #self.odom_transform.child_frame_id = 'odom'
        self.odom_transform.transform.translation.x = easting
        self.odom_transform.transform.translation.y = northing
        self.odom_transform.transform.translation.z = altitude
        self.odom_transform.transform.rotation.w = 1.0  # Identity rotation

        # Publish the transforms immediately once they are determined
        self._log(f"Set static transform map → odom at UTM ({easting:.2f}, {northing:.2f}) UTM_{zone}{letter}")
        self.static_broadcaster.sendTransform([self.base_transform, self.odom_transform])

        self.origin_set = True

    def transform_timer(self):
        # Republish transform using a timer
        if not self.origin_set:
            return

        if self.base_transform is None or self.odom_transform is None:
            self._log(f"Invalid base or odom transform")
            return

        if self.verbose:
            self._log("Broadcasting base and odom transforms")

        self.static_broadcaster.sendTransform([self.base_transform, self.odom_transform])

    def _log(self, message):
        self.get_logger().info(message)


def main(args=None, namespace=None):
    rclpy.init(args=args)
    node = MapOdomInitializer(namespace=namespace)

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
