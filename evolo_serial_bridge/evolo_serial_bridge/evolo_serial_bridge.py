
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

from evolo_msgs.msg import Topics as evoloTopics

import random

import serial





class Mqtt_bridge(Node):

    def __init__(self):
        super().__init__('read_and_publish')


        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # open serial port

        # Create ROS publisher
        self.publisher_ = self.create_publisher(String, evoloTopics.EVOLO_CAPTAIN_FROM, 10)
        
        # Create ROS subscriber
        self.subscription = self.create_subscription(String,evoloTopics.EVOLO_CAPTAIN_TO, self.ros_callback,10)
        self.subscription # prevent unused variable warning

        self.serial_thread = Thread(target= self.run)
        self.serial_thread.start()

    def ros_callback(self, msg:String):
        self.get_logger().info(f"Received ROS message '{msg}'")
        self.ser.write(msg.data.encode())
        self.ser.write("\r\n".encode())
        


    def run(self):
        while rclpy.ok():
            try:
                line = self.ser.readline()   # read a '\n' terminated line
                if(len(line) == 0):
                    continue
                self.get_logger().info(f"Read line from serial '{line}'")
                msg = String()
                msg.data = line.decode()
                self.publisher_.publish(msg)
            except Exception as e:
                print(e)








def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = Mqtt_bridge()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()