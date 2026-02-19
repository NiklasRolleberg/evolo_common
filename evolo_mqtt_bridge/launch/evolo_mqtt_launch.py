from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics

#MQTT parameters
broker_addr = '127.0.0.01'
broker_port = 1883
broker_uname = ""
broker_pw = ""

#Topic parameters
ros_publish_topic = evoloTopics.EVOLO_CAPTAIN_FROM
ros_subscribe_topic = evoloTopics.EVOLO_CAPTAIN_TO
mqtt_publish_topic =  ""
mqtt_subscribe_topics = [""]


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
        name="evolo_mqtt_bridge",
        parameters=[{'use_sim_time': use_sim_time,
                     "broker_addr" : broker_addr,
                    "broker_port" : broker_port,
                    "broker_uname" : broker_uname,
                    "broker_pw" : broker_pw,
                    "ros_publish_topic" : ros_publish_topic,
                    "ros_subscribe_topic" : ros_subscribe_topic,
                    "mqtt_publish_topic" : mqtt_publish_topic,
                    "mqtt_subscribe_topics" : mqtt_subscribe_topics
                    }])

    captain_interface = Node(
        package='evolo_captain_interface',
        namespace=robot_ns,
        executable='interface',
        name="captain_interface",
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )


    return LaunchDescription([
        robot_ns_launch_arg,
        sim_time_arg, 
        mqtt_bridge_node,
        captain_interface
    ])
