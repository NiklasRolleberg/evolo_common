
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

from evolo_msgs.msg import Topics as evoloTopics

import random


#from paho.mqtt import client as mqtt_client
import paho.mqtt.client as mqtt
import json

broker = 'xxx.xxx.xxx.xxx'
port = 0
username = ''
password = ''


# Generate a Client ID with the subscribe prefix.
client_id = f'subscribe-{random.randint(0, 100)}'



#Mqtt topics to subscribe and publish to
mqtt_sub_topic  = "topic"
mqtt_pub_topic  = "topic"




class Mqtt_bridge(Node):

    def __init__(self):
        super().__init__('read_and_publish')

        self.mqtt_client = None

        # Create ROS publisher
        self.publisher_ = self.create_publisher(String, evoloTopics.EVOLO_CAPTAIN_FROM, 10)
        
        # Create ROS subscriber
        self.subscription = self.create_subscription(String,evoloTopics.EVOLO_CAPTAIN_TO, self.ros_callback,10)
        self.subscription # prevent unused variable warning

        self.mqtt_thread = Thread(target= self.run)
        self.mqtt_thread.start()

    def ros_callback(self, msg):
        self.get_logger().info("Received ROS message '{msg}'")
        if(self.mqtt_client != None):
            self.mqtt_client.publish(topic=mqtt_pub_topic, payload=msg.data.encode())

    
    def mqtt_callback(self,client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        ros_msg = String()
        ros_msg.data = msg.payload.decode()
        self.publisher_.publish(ros_msg)

    def mqtt_connect(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        #client = mqtt_client.Client(client_id)
        client = mqtt.Client(client_id=client_id, clean_session=True, userdata=None)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client


    def mqtt_subscribe(self,client, topic):
        client.subscribe(topic)
        client.on_message = self.mqtt_callback


    def run(self):
        self.mqtt_client = self.mqtt_connect()
        self.mqtt_subscribe(self.mqtt_client, mqtt_sub_topic)
        self.mqtt_client.loop_forever()






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