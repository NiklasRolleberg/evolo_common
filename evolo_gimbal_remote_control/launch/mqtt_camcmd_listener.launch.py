from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_msgs.msg import Topics as evoloTopics

#MQTT parameters
broker_addr = '127.0.0.1'
broker_port = 1883
broker_uname = ""
broker_pw = ""

#Topic parameters
ros_publish_topic = evoloTopics.EVOLO_MQTT_GIMBAL_CAMCMD
mqtt_subscribe_topics = ["e/to/3843318481/camcmd"]


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )
    
    mqtt_bridge_node = Node(
        package='evolo_mqtt_bridge',
        namespace=robot_ns,
        executable='bridge',
        name="evolo_mqtt_camcmd_bridge",
        parameters=[{'use_sim_time': use_sim_time,
                     "broker_addr" : broker_addr,
                    "broker_port" : broker_port,
                    "broker_uname" : broker_uname,
                    "broker_pw" : broker_pw,
                    "ros_publish_topic" : ros_publish_topic,
                    "ros_subscribe_topic" : "not_used",
                    "mqtt_subscribe_topics" : mqtt_subscribe_topics
                    }])

    return LaunchDescription([
        robot_ns_launch_arg,
        sim_time_arg, 
        mqtt_bridge_node,
    ])
