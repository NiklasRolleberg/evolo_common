
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

from evolo_msgs.msg import Topics as evoloTopics

import random


#from paho.mqtt import client as mqtt_client
import paho.mqtt.client as mqtt
import json



class Mqtt_bridge(Node):

    def __init__(self):
        super().__init__('read_and_publish')

        #Settings for mqtt connection
        self.declare_parameter('broker_addr', '127.0.0.1')
        self.broker = self.get_parameter('broker_addr').value
        
        self.declare_parameter('broker_port', 1883)
        self.port = self.get_parameter('broker_port').value
        
        self.declare_parameter('broker_uname', '')
        self.username = self.get_parameter('broker_uname').value

        self.declare_parameter('broker_pw', '')
        self.password = self.get_parameter('broker_pw').value

        #Settings for topics
        self.declare_parameter('ros_publish_topic', evoloTopics.EVOLO_CAPTAIN_FROM)
        self.ROS_publish_topic = self.get_parameter('ros_publish_topic').value
        
        self.declare_parameter('ros_subscribe_topic', evoloTopics.EVOLO_CAPTAIN_TO)
        self.ROS_subscribe_topic = self.get_parameter('ros_subscribe_topic').value

        self.declare_parameter('mqtt_publish_topic', "pub")
        self.MQTT_publish_topic = self.get_parameter('mqtt_publish_topic').value

        self.declare_parameter('mqtt_subscribe_topics', ["sub"])
        self.MQTT_subscribe_topics = self.get_parameter('mqtt_subscribe_topics').value

        #print(f"broker `{self.broker}` topics: {self.MQTT_subscribe_topics}")
        self.get_logger().info(f"broker `{self.broker}` topics: {self.MQTT_subscribe_topics}")

        # Generate a Client ID with the subscribe prefix.
        self.mqtt_client = None

        # Create ROS publisher
        self.publisher_ = self.create_publisher(String, self.ROS_publish_topic, 10)
        
        # Create ROS subscriber
        self.subscription = self.create_subscription(String,self.ROS_subscribe_topic, self.ros_callback,10)
        self.subscription # prevent unused variable warning

        self.mqtt_thread = Thread(target= self.run)
        self.mqtt_thread.start()

    def ros_callback(self, msg):
        self.get_logger().info("Received ROS message '{msg}'")
        if(self.mqtt_client != None):
            self.mqtt_client.publish(topic=self.MQTT_publish_topic, payload=msg.data.encode())

    
    def mqtt_callback(self,client, userdata, msg):
        #self.get_logger().info(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        ros_msg = String()
        ros_msg.data = msg.payload.decode()
        self.publisher_.publish(ros_msg)

    def mqtt_connect(self):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client_id = f'subscribe-{random.randint(0, 100)}'
        client = mqtt.Client(client_id=client_id, clean_session=True, userdata=None)
        client.username_pw_set(self.username, self.password)
        client.on_connect = on_connect
        client.connect(self.broker, self.port)
        return client


    def mqtt_subscribe(self,client, topic):
        client.subscribe(topic)
        client.on_message = self.mqtt_callback


    def run(self):
        self.mqtt_client = self.mqtt_connect()
        for topic in self.MQTT_subscribe_topics:
            self.mqtt_subscribe(self.mqtt_client, topic)
            self.get_logger().info(f"subscribed to `{topic}`")
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