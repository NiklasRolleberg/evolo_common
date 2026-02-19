
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from threading import Thread

from evolo_msgs.msg import Topics as evoloTopics
from evolo_msgs.msg import CaptainState

import random
import json





class Evolo_captain_interface(Node):

    def __init__(self):
        super().__init__('Evolo_captain_interface')

        #control sepoints
        self.speed_setpoint = None
        self.speed_ts = 0
        self.turning_setpoint = None
        self.turning_ts = 0
        
        #Message object
        self.msg = CaptainState()

        # Create ROS publisher
        self.state_publisher_ = self.create_publisher(CaptainState, evoloTopics.EVOLO_CAPTAIN_STATE, 10)
        self.setpoint_publisher = self.create_publisher(String, "temp", 10)
        
        # Create ROS subscriber
        self.subscription = self.create_subscription(String,evoloTopics.EVOLO_CAPTAIN_FROM, self.captain_callback,10)
        self.subscription # prevent unused variable warning

        self.steering_sub = self.create_subscription(Float32, evoloTopics.EVOLO_STEERING_SETPOINT, self.steer_control_callback,10)
        self.speed_sub = self.create_subscription(Float32, evoloTopics.EVOLO_SPEED_SETPOINT, self.speed_control_callback,10)

        timer_period = 1/2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def time_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def timer_callback(self):
        now = self.time_now()
        turning_OK =  self.turning_ts is not None and now-self.turning_ts < 1 and self.turning_setpoint is not None #Turing setpoint is OK
        speed_OK = self.speed_ts is not None and now-self.speed_ts < 1 and self.speed_setpoint is not None #Speed setpoint is OK

        if(turning_OK and speed_OK):
            msg = String()
            setpoints = {"bts":self.turning_setpoint, "sog":self.speed_setpoint}
            msg.data = json.dumps({"backseat": setpoints})
            self.setpoint_publisher.publish(msg)
            self.get_logger().info(f"Published:  '{msg.data}'")
        else:
            msg = String()
            setpoints = {"bts":0, "sog":0}
            msg.data = json.dumps({"backseat": setpoints})
            self.setpoint_publisher.publish(msg)
            self.get_logger().info(f"Published:  '{msg.data}'")

    def steer_control_callback(self, msg):
        self.turning_setpoint = msg.data #TODO convert from ENU to NED
        self.turning_ts = self.time_now()
        
    def speed_control_callback(self, msg):
        self.speed_setpoint = msg.data #TODO convert from m/s to knots
        self.speed_ts = self.time_now()


    def captain_callback(self, msg:String):        
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"msg received: {msg.data}")

            #parse highAll
            if("ts" in data.keys() and
                "hBits" in data.keys() and
                "hPerc" in data.keys() and
                "sBits" in data.keys() and
                "throt" in data.keys() and
                "lat" in data.keys() and
                "lon" in data.keys() and
                "sog" in data.keys() and
                "cog" in data.keys() and
                "hdop" in data.keys() and
                "cef" in data.keys() and
                "elev" in data.keys() and
                "ail" in data.keys() and
                "altAim" in data.keys() and
                "pitchAim" in data.keys() and
                "rollAim" in data.keys() and
                "sogAim" in data.keys() and
                "rud" in data.keys() and
                "tLeft" in data.keys() and
                "tRight" in data.keys()) :
                self.get_logger().info(f"highAll message")
                
                try:
                    self.msg.ms = float(data["ts"])
                    #self.msg.hBits = float(data["hBits"])
                    #self.msg.hPerc = float(data["hPerc"])
                    #self.msg.sBits = float(data["sBits"])
                    #self.msg.throt = float(data["throt"])
                    self.msg.lat = float(data["lat"])
                    self.msg.lon = float(data["lon"])
                    self.msg.sog = float(data["sog"])
                    self.msg.cog = float(data["cog"])
                    self.msg.hdop = float(data["hdop"])
                    self.msg.cef = float(data["cef"])
                    #self.msg.elev = float(data["elev"])
                    #self.msg.ail = float(data["ail"])
                    #self.msg.altAim = float(data["altAim"])
                    #self.msg.pitchAim = float(data["pitchAim"])
                    #self.msg.rollAim = float(data["rollAim"])
                    #self.msg.sogAim = float(data["sogAim"])
                    #self.msg.rud = float(data["rud"])
                    #self.msg.tLeft = float(data["tLeft"])
                    #self.msg.tRight = float(data["tRight"])
                    #self.get_logger().info(f"Publishing state message {self.msg}")
                    self.state_publisher_.publish(self.msg)
                except Exception as e:
                    self.get_logger().info(f"Error {e}")    
                
            


            '''
            if("state" in data.keys()):
                state = data["state"]
                #self.get_logger().info(f"Evolo state: {state}")


                self.msg.lat = float(state[0])
                self.msg.lon = float(state[1])
                self.msg.sog = float(state[2])
                self.msg.cog = float(state[3])
                self.msg.roll = float(state[4])
                self.msg.pitch = float(state[5])
                self.msg.ms = float(state[6])
                self.msg.freq = float(state[7])
                self.msg.hdop = float(state[8])
                self.msg.health_percent = float(state[9])
                self.msg.mode = float(state[10])
                self.msg.cef = float(state[11])
                self.msg.voltage = float(state[12])
                self.msg.current = float(state[13])
                self.msg.percent_throttle = float(state[14])
                self.msg.altitude = float(state[15])
                self.msg.virtual_elevator = float(state[16])
                self.msg.virtual_aileron = float(state[17])
                self.msg.alt_aim = float(state[18])
                self.msg.healt_bits = state[19]
                self.msg.elevator_bias = float(state[20])
                self.msg.aileron_bias = float(state[21])
                self.msg.status_bits = state[22]
                self.msg.hs = float(state[23])
                self.msg.pitch_aim = float(state[24])
                self.msg.roll_aim = float(state[25])
                self.msg.sog_aim = float(state[26])
                self.msg.four_g_data = float(state[27])
                self.msg.free_mbon_sd = float(state[28])
                self.msg.speed_limit = float(state[29])
                self.msg.left_throttle_couch = float(state[30])
                self.msg.right_throttle_couch = float(state[31])
                self.msg.status_bits2 = state[32]
                self.msg.radar1_confidence = float(state[33])
                self.msg.radar2_confidence = float(state[34])
                self.msg.roll_rate = float(state[35])
                self.msg.pitch_rate = float(state[36])

                # Health bits
                #bits = msg.healt_bits
                #self.msg.health_loop_freq = (bits & 0b0000000000000001) != 0
                #self.msg.health_voltage = (bits & 0b0000000000000010) != 0
                #self.msg.health_hdop = (bits & 0b0000000000000100) != 0
                #self.msg.health_gps_rate = (bits & 0b0000000000001000) != 0
                #self.msg.health_imu = (bits & 0b0000000000010000) != 0
                #self.msg.health_radar = (bits & 0b0000000000100000) != 0
                #self.msg.health_tilt = (bits & 0b0000000001000000) != 0
                #self.msg.health_radar_x = (bits & 0b0000000010000000) != 0
                #self.msg.health_pitch = (bits & 0b0000000100000000) != 0
                #self.msg.health_roll = (bits & 0b0000001000000000) != 0
                #self.msg.health_ax = (bits & 0b0000010000000000) != 0
                #self.msg.health_throttle = (bits & 0b0000100000000000) != 0
                #self.msg.health_time_lock = (bits & 0b0001000000000000) != 0
                #self.msg.health_hil = (bits & 0b0010000000000000) != 0
                #self.msg.health_sd_memory = (bits & 0b0100000000000000) != 0
                #self.msg.health_network_timeout = (bits & 0b1000000000000000) != 0#
#


                # StatusBits
                bits = msg.status_bits
                #self.msg.status_throttle_disarmed = (bits & 0b0000000000000001) != 0
                #self.msg.status_throttle_manual = (bits & 0b0000000000000010) != 0
                #self.msg.status_throttle_auto = (bits & 0b0000000000000100) != 0
                #self.msg.status_guidance_manual = (bits & 0b0000000000001000) != 0
                #self.msg.status_guidance_target = (bits & 0b0000000000010000) != 0
                #self.msg.status_guidance_route = (bits & 0b0000000000100000) != 0
                #self.msg.status_paused = (bits & 0b0000000001000000) != 0
                #self.msg.status_hil = (bits & 0b0000000010000000) != 0
                #self.msg.status_sim = (bits & 0b0000000100000000) != 0
                #self.msg.status_telemetry_4g = (bits & 0b0000001000000000) != 0
                #self.msg.status_rc_required = (bits & 0b0000010000000000) != 0
                #self.msg.status_route_finished = (bits & 0b0000100000000000) != 0
                #self.msg.status_route_looping = (bits & 0b0001000000000000) != 0
                #self.msg.status_route_ongoing = (bits & 0b0010000000000000) != 0
                #self.msg.status_joystick_present = (bits & 0b0100000000000000) != 0
                #self.msg.status_rc_active = (bits & 0b1000000000000000) != 0
                #self.msg.status_radar1_active = (bits & 0b10000000000000000) != 0
                #self.msg.status_radar2_active = (bits & 0b100000000000000000) != 0
                #self.msg.status_gamepad_xxx = (bits & 0b1000000000000000000) != 0
                #self.msg.status_allow_backseat_ctrl = (bits & 0b10000000000000000000) != 0

                self.publisher_.publish(msg)
                '''

        except Exception as e:
            print(e)

        #TODO parse message







def main(args=None):

    rclpy.init(args=args)

    captain_interface = Evolo_captain_interface()

    rclpy.spin(captain_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    captain_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()