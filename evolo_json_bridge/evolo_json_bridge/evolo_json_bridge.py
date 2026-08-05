import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from evolo_msgs.msg import Topics as evoloTopics
from sbg_driver.msg import SbgEkfNav
from smarc_msgs.msg import Sidescan
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Duration, Time
from geodesy import utm
from geographic_msgs.msg import GeoPoint

import json
import yaml


class Translator(Node):

    def __init__(self):
        super().__init__('mqtt_translator')

        #Settings for topics

        # Publish topics
        self.declare_parameter('status_publish_topic', "/evolo/node_red/status")
        self.status_publish_topic = self.get_parameter('status_publish_topic').value 

        self.declare_parameter('sidescan_publish_topic', "/evolo/node_red/sidescan")
        self.sss_publish_topic = self.get_parameter('sidescan_publish_topic').value

        self.declare_parameter('wp_publish_topic', "waraps/sensor/current_wp")
        self.wp_publish_topic = self.get_parameter('wp_publish_topic').value

        #Subscribte topics
        # SBG
        # Sidescan

        self.declare_parameter('sbg_ekf_nav_topic', "/evolo/sbg/ekf_nav")
        self.sbg_nav_subscribe_topic = self.get_parameter('sbg_ekf_nav_topic').value
        
        self.declare_parameter('sss_topic', "/payload/sidescan")
        self.sss_topic = self.get_parameter('sss_topic').value

        self.declare_parameter('current_wp_topic', evoloTopics.EVOLO_CURRENT_WP)
        self.current_wp_topic = self.get_parameter('current_wp_topic').value

        # Tf listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=True
        )
        self.root_tf_frame = None

        #messages
        self.sbg_ekf_nav_msg = SbgEkfNav()
        self.sbg_ekf_nav_msg_time = self.time_now()

        self.sss_msg = Sidescan()
        self.sss_msg_time = self.time_now()

        self.wp_message = PointStamped()
        self.wp_message_time = self.time_now()

        # Create ROS publishers
        self.status_publisher_ = self.create_publisher(String, self.status_publish_topic, 10)
        self.sidescan_publisher_ = self.create_publisher(String, self.sss_publish_topic, 10)
        self.wp_publisher_ = self.create_publisher(String, self.wp_publish_topic, 10)
        
        # Create ROS subscriber
        self.sbg_ekf_subscription = self.create_subscription(SbgEkfNav,self.sbg_nav_subscribe_topic, self.sbg_ekf_nav_callback,10)
        self.sbg_ekf_subscription # prevent unused variable warning

        self.sss_subscription = self.create_subscription(Sidescan,self.sss_topic, self.sss_callback,10)

        self.wp_subscription = self.create_subscription(PointStamped, self.current_wp_topic, self.wp_callback,10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def time_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sbg_ekf_nav_callback(self, msg):
        self.sbg_ekf_nav_msg = msg
        self.sbg_ekf_nav_msg_time = self.time_now()

    def sss_callback(self, msg : Sidescan):
        self.sss_msg = msg
        self.sss_msg_time = self.time_now()

        #Convert sidescan message and republish
        range = 1500.0 * self.sss_msg.max_duration if self.sss_msg.max_duration != 0 else 100.0
        frequency = self,self.sss_msg.frequency_id*10 if self.sss_msg.frequency_id != 0 else 200
        json_data = {"range": range,
                     "frequency":frequency,
                     "starboard_channel": [int(i) for i in self.sss_msg.starboard_channel], 
                     "port_channel": [int(i) for i in self.sss_msg.port_channel]}
        
        msg = String()
        msg.data = json.dumps({"sidescan":json_data})
        self.sidescan_publisher_.publish(msg)

    def wp_callback(self, msg):
        self.wp_message = msg
        self.wp_message_time = self.time_now()


    def timer_callback(self):

        #SBG solution info
        # 0 UNINITIALIZED	The Kalman filter is not initialized and the returned data are all invalid.
        # 1 VERTICAL_GYRO	The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely.
        # 2 AHRS			A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely.
        # 3 NAV_VELOCITY	The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation.
        # 4 NAV_POSITION	Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.
        if(self.time_now() - self.sbg_ekf_nav_msg_time > 1.0):
            sbg_status = "timeout"
        elif self.sbg_ekf_nav_msg.status.solution_mode == 0:
            sbg_status = "UNINITIALIZED"
        elif self.sbg_ekf_nav_msg.status.solution_mode == 1:
            sbg_status = "VERTICAL_GYRO"
        elif self.sbg_ekf_nav_msg.status.solution_mode == 2:
            sbg_status = "AHRS"
        elif self.sbg_ekf_nav_msg.status.solution_mode == 3:
            sbg_status = "NAV_VELOCITY"
        elif self.sbg_ekf_nav_msg.status.solution_mode == 4:
            sbg_status = "NAV_POSITION"
        else:
            sbg_status = "Error"

        #SBG aligned
        sbg_aligned = "YES" if self.sbg_ekf_nav_msg.status.align_valid else "NO"

        #SBG heading valid
        sbg_heading_valid = "YES" if self.sbg_ekf_nav_msg.status.heading_valid else "NO"

        #SBG heading used
        sbg_heading_used = "YES" if self.sbg_ekf_nav_msg.status.gps1_hdt_used else "NO"

        #Sidescan
        sidescan_ok = "YES" if self.time_now() - self.sss_msg_time < 2 else "NO"

        data = {"sbg_status": sbg_status, 
                "sbg_aligned": sbg_aligned,
                "sbg_heading_valid": sbg_heading_valid,
                "sbg_heading_used": sbg_heading_used,
                "sidescan_ok: ": sidescan_ok
                }

        msg = String()
        msg.data = json.dumps({"scientist_status": data})
        self.status_publisher_.publish(msg)

        #current wp

        #No message. Return
        if(self.wp_message == None):
            return
        
        #Message is old. Return
        if(self.time_now() - self.wp_message_time > 1):
            return

        #Check for root frame
        if self.root_tf_frame == None:
            self.root_tf_frame = self.find_root()

        print("root tf frame: " + str(self.root_tf_frame))

        if self.root_tf_frame == None:
            return

        #convert to global coordinates
        try:
            u, zone, band = self.root_tf_frame.split("_")
            zone = int(zone)

            #Transform point to root frame
            t = self._tf_buffer.lookup_transform(
                target_frame=self.root_tf_frame,
                source_frame=self.wp_message.header.frame_id,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
            wp_in_utm_frame : PointStamped = do_transform_point(self.wp_message, t)

            point = utm.UTMPoint(easting=wp_in_utm_frame.point.x, northing=wp_in_utm_frame.point.y,
                 altitude=0.0, zone=zone, band=band)
            
            geo_point : GeoPoint = point.toMsg()

            print("geopoint: " + str(geo_point))

            #Convert geopoint to Json and publish
            d = {}
            d["altitude"] = geo_point.altitude
            d["latitude"] = geo_point.latitude
            d["longitude"] = geo_point.longitude
            d["rostype"] = "GeoPoint"

            m = String()
            m.data = json.dumps(d)
            self.wp_publisher_.publish(m)
            

        except Exception as e:
            print(e)
        

        
        #reset wp message
        self.wp_message = None


    def find_root(self) -> str | None:
        """Parse the buffer's YAML once to find the root."""
        raw = self._tf_buffer.all_frames_as_yaml()
        frames = yaml.safe_load(raw)
        if not frames:
            return None

        all_children = set(frames.keys())
        all_parents  = {info['parent'] for info in frames.values() if info.get('parent')}
        roots = all_parents - all_children
        root_list = list(roots)
        if(len(root_list) > 1): #Disconnected tf tree
            print("Error bad TF tree. Fix it")
            return None
        if(len(root_list) == 1):
            return root_list[0]
        print("No root found")
        return None

def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = Translator()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()