#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from datetime import datetime, timezone

from sbg_driver.msg import (
    SbgGpsPos,
    SbgGpsVel,
    SbgUtcTime,
)

from std_msgs.msg import String


class SbgJsonPublisher(Node):

    def __init__(self):
        super().__init__('sbg_gps_relay')

        self.latitude = None
        self.longitude = None

        self.vel_north = None
        self.vel_east = None

        self.gps_stamp = None

        self.create_subscription(
            SbgGpsPos,
            '/evolo/sbg/gps_pos',
            self.gps_pos_callback,
            10)

        self.create_subscription(
            SbgGpsVel,
            '/evolo/sbg/gps_vel',
            self.gps_vel_callback,
            10)

        self.json_pub = self.create_publisher(
            String,
            '/evolo/captain/to',
            10)

        self.create_timer(0.1, self.publish_json)

    def gps_pos_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.gps_stamp = msg.header.stamp

    def gps_vel_callback(self, msg):
        self.vel_north = msg.velocity.x
        self.vel_east = msg.velocity.y

    def publish_json(self):

        if None in (
            self.latitude,
            self.longitude,
            self.vel_north,
            self.vel_east,
            self.gps_stamp
        ):
            return
        
        #TODO check if GPS is valid

        sog = 1.943844*math.sqrt(
            self.vel_north**2 +
            self.vel_east**2
        )

        cog = math.degrees(
            math.atan2(
                self.vel_east,
                self.vel_north
            )
        )

        if cog < 0:
            cog += 360.0

        timestamp = (
            self.gps_stamp.sec +
            self.gps_stamp.nanosec * 1e-9
        )

        dt = datetime.fromtimestamp(
            timestamp,
            tz=timezone.utc
        )

        data = {
            "sogkts": round(sog, 3),
            "cog": round(cog, 3),
            "lat": self.latitude,
            "lon": self.longitude,
            "y": dt.year,
            "m": dt.month,
            "d": dt.day,
            "h": dt.hour,
            "m": dt.minute,
            "s": dt.second
        }

        msg = String()
        msg.data = "{\"gpsPeripheral\":" + json.dumps(data) +"}"

        self.json_pub.publish(msg)

        #Clear
        #print("reset")
        self.latitude = None
        self.longitude = None
        self.vel_north = None
        self.vel_east = None
        self.gps_stamp = None


def main():
    rclpy.init()

    node = SbgJsonPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()