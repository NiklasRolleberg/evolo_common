import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from evolo_msgs.msg import Topics as evoloTopics
from sbg_driver.msg import SbgEkfNav
from smarc_msgs.msg import Sidescan

import json



class Translator(Node):

    def __init__(self):
        super().__init__('mqtt_translator')

        #Settings for topics

        # Publish topics
        self.declare_parameter('status_publish_topic', "/evolo/node_red/status")
        self.status_publish_topic = self.get_parameter('status_publish_topic').value 

        self.declare_parameter('sidescan_publish_topic', "/evolo/node_red/sidescan")
        self.sss_publish_topic = self.get_parameter('sidescan_publish_topic').value

        #Subscribte topics
        # SBG
        # BT feedback
        # Sidescan

        self.declare_parameter('sbg_ekf_nav_topic', "/evolo/sbg/ekf_nav")
        self.sbg_nav_subscribe_topic = self.get_parameter('sbg_ekf_nav_topic').value

        self.declare_parameter('BT_executing_tasks', "/evolo/waraps/sensor/executing_tasks")
        self.BT_executing_tasks_topic = self.get_parameter('BT_executing_tasks').value
        
        self.declare_parameter('sss_topic', "/payload/sidescan")
        self.sss_topic = self.get_parameter('sss_topic').value

        #messages
        self.sbg_ekf_nav_msg = SbgEkfNav()
        self.sbg_ekf_nav_msg_time = self.time_now()

        self.BT_executing_tasks_msg = String()
        self.BT_executing_tasks_msg_time = self.time_now()

        self.sss_msg = Sidescan()
        self.sss_msg_time = self.time_now()

        # Create ROS publishers
        self.status_publisher_ = self.create_publisher(String, self.status_publish_topic, 10)
        self.sidescan_publisher_ = self.create_publisher(String, self.sss_publish_topic, 10)
        
        # Create ROS subscriber
        self.sbg_ekf_subscription = self.create_subscription(SbgEkfNav,self.sbg_nav_subscribe_topic, self.sbg_ekf_nav_callback,10)
        self.sbg_ekf_subscription # prevent unused variable warning

        self.BT_subscription = self.create_subscription(String,self.BT_executing_tasks_topic, self.bt_executing_tasks_callback,10)
        self.BT_subscription # prevent unused variable warning

        self.sss_subscription = self.create_subscription(Sidescan,self.sss_topic, self.sss_callback,10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def time_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sbg_ekf_nav_callback(self, msg):
        self.sbg_ekf_nav_msg = msg
        self.sbg_ekf_nav_msg_time = self.time_now()

    def bt_executing_tasks_callback(self, msg):
        self.BT_executing_tasks_msg = msg
        self.BT_executing_tasks_msg_time = self.time_now()

    def sss_callback(self, msg : Sidescan):
        self.sss_msg = msg
        self.sss_msg_time = self.time_now()

        #Convert sidescan message and republish

        json_data = {"range":1500.0 * self.sss_msg .max_duration,
                     "frequency":200,
                     "starboard_channel": [int(i) for i in self.sss_msg.starboard_channel], 
                     "port_channel": [int(i) for i in self.sss_msg.port_channel]}
        
        msg = String()
        msg.data = json.dumps({"sidescan":json_data})
        self.sidescan_publisher_.publish(msg)


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

        #TODO get lidar status from health monitor

        #TODO get camera status from health monitor

        data = {"sbg_status": sbg_status, 
                "sbg_aligned": sbg_aligned,
                "sbg_heading_valid": sbg_heading_valid,
                "sbg_heading_used": sbg_heading_used,
                "sidescan_ok: ": sidescan_ok,
                "lidar_ok": "not checked",
                "camera_ok": "not checked"}

        msg = String()
        msg.data = json.dumps({"scientist_status": data})
        self.status_publisher_.publish(msg)


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