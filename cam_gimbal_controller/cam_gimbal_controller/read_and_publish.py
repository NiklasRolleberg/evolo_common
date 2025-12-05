
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from cam_gimbal_msgs.msg import Gcudata

import socket
import binascii
import time
import struct

GCU_IP = "192.168.2.210"
TCP_PORT = 2332
RECONNECT_DELAY = 5  # seconds

SOME_MIN_LENGTH = 10

sock = None

class GimbalReadAndPublish(Node):

    def __init__(self):
        super().__init__('read_and_publish')

        # Create publisher
        self.publisher_ = self.create_publisher(Gcudata, 'cam_gcu_data', 10)
        timer_period = 1/50  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create subscriber
        self.subscription = self.create_subscription(Vector3,"desired_gimbal_euler",self.listener_callback,10 )
        self.subscription # prevent unused variable warning

    def timer_callback(self):
        msg = Gcudata()
        data_from_camera = send_null_command()

        # Store operating mode
        extracted_operating_mode = struct.unpack("B", data_from_camera[5:6])
        msg.operating_mode = extracted_operating_mode[0]

        # Store relative angle and convert to Evolos Coordinate system
        extracted_relative_angle = struct.unpack("<hhh",data_from_camera[12:18])
        msg.relative_pitch = extracted_relative_angle[1] / 100.0
        msg.relative_roll = extracted_relative_angle[0] / 100.0
        msg.relative_yaw = extracted_relative_angle[2] / -100.0

        # Store absolute angle 
        extracted_absolute_angle = struct.unpack("<hhh",data_from_camera[18:24])
        msg.absolute_pitch = extracted_absolute_angle[0] / 100.0
        msg.absolute_roll = extracted_absolute_angle[1] / 100.0
        msg.absolute_yaw = extracted_absolute_angle[2] / 100.0

        # Store error code
        extracted_error_code = struct.unpack("<h", data_from_camera[41:43])
        msg.error_code = extracted_error_code[0]

        # Store camera status
        extracted_camera_status = struct.unpack("<h", data_from_camera[64:66])
        msg.osd = bool(getBit(extracted_camera_status[0], 13)) # B13: 0 - OSD off, 1 - OSD on
        msg.recording = bool(getBit(extracted_camera_status[0], 4)) # B4: 0 - not recording, 1 - recording

        print(msg)
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        print(f"desired_gimbal_euler received: x:{msg.x}, {msg.y}, {msg.z}") # print for debugging
        send_euler_command(int(msg.x), int(msg.y), int(msg.z)) # Send euler order
        send_null_command() # Seperate orders with a null command (required by gimbal)

# This function returns the value of bit n
def getBit(value, n):
    return (value >> n) & 1



def build_packet(
    order: int,
    param_bytes: bytes = b"",
    roll: int = 0,
    pitch: int = 0,
    yaw: int = 0,
    ctrl_valid: bool = False,
) -> bytes:
    main = bytearray(32)
    main[0:2] = roll.to_bytes(2, "little", signed=True)
    main[2:4] = pitch.to_bytes(2, "little", signed=True)
    main[4:6] = yaw.to_bytes(2, "little", signed=True)
    main[6] = 0x04 if ctrl_valid else 0x00
    main[25] = 0x01

    sub = bytearray(32)

    header = b"\xa8\xe5"
    version = b"\x02"
    order_byte = bytes([order])
    packet = header + b"\x00\x00" + version + main + sub + order_byte + param_bytes

    total_len = len(packet) + 2
    packet = packet[:2] + total_len.to_bytes(2, "little") + packet[4:]

    crc = binascii.crc_hqx(packet, 0)
    full_packet = packet + crc.to_bytes(2, "big")

    # Log full packet in hex for debugging
    # print("SEND:", full_packet.hex(" ").upper())

    return full_packet

def establish_connection_with_handshake():
    global sock
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((GCU_IP, TCP_PORT))
            print("TCP socket connected, sending test command...")

            # Send a null command that the gimbal should respond to
            null_packet = build_packet(order=0x00)
            sock.sendall(null_packet)

            # Attempt to read the response
            response = sock.recv(1024)
            # Here, you'll have to decode the response or check certain bytes
            # For example, if you expect a certain length or header:
            if not response or len(response) < SOME_MIN_LENGTH:
                raise Exception("Handshake failed: no or invalid response")

            # If you get here, you have a valid response -> handshake is successful
            print(f"Connected to GCU at {GCU_IP}:{TCP_PORT} (handshake confirmed)")
            return  # The global sock is valid now

        except Exception as e:
            print(f"Connection/handshake failed: {e}")
            if sock:
                try:
                    sock.close()
                except Exception as close_e:
                    print(f"Error closing socket: {close_e}")
            time.sleep(RECONNECT_DELAY)

# Send a null-command, and return the response 
def send_null_command():
    global sock
    if sock:
        null_packet = build_packet(order=0x00)
        sock.sendall(null_packet)
        return sock.recv(1024)

# Sends euler-command
# Notice: Evolo and the gimbal uses different coordinate systems
#         Evolo's coordinate system is used in the parameters, to later be converted to the gimbal system.
def send_euler_command(roll: int, pitch: int, yaw: int):
    global sock
    if sock:
        send_null_command()
        packet = build_packet(
            order=0x10,
            # Converts to gimbal coordinate system and deci-degrees.
            roll=int(roll) * 100,
            pitch=int(pitch) * -100,
            yaw=int(yaw) * -100,
            ctrl_valid=True,
        )
        sock.sendall(packet)
        sock.recv(1024)

def send_toggle_record_command():
    global sock
    if sock:
        send_null_command()
        packet = build_packet(
            order=0x21,
            param_bytes=b"/x01",
            ctrl_valid=True,
        )
        sock.sendall(packet)
        return sock.recv(1024)



def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = GimbalReadAndPublish()

    establish_connection_with_handshake()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()