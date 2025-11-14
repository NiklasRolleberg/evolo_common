# This script is used for testing read_and_publish.py

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

import math


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('order_test_euler')
        self.publisher_ = self.create_publisher(Vector3, 'desired_gimbal_euler', 10)
        self.timer_period = 1/50  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Vector3()
        if(self.i * self.timer_period < 2*math.pi):
            msg.x = 45*math.sin(self.i * self.timer_period)
            msg.y = 0.0
            msg.z = 0.0
        elif (self.i * self.timer_period < 4*math.pi):
            msg.x = 0.0
            msg.y = 45*math.sin(self.i * self.timer_period)
            msg.z = 0.0
        elif (self.i * self.timer_period < 6*math.pi):
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 45*math.sin(self.i * self.timer_period)
        else: 
            self.i = 0

        self.publisher_.publish(msg)

        print("Published order")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()