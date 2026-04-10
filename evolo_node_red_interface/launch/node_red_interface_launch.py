from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics

#MQTT parameters
broker_addr = '127.0.0.1'
broker_port = 1883
broker_uname = ""
broker_pw = ""


# topics for translator to publish to 
status_publish_topic = "/evolo/node_red/status"
sidescan_publish_topic = "/evolo/node_red/sidescan"

#topics for translator to subscribe to
sbg_ekf_nav_topic = "/evolo/sbg/ekf_nav"
BT_executing_tasks = "/evolo/waraps/sensor/executing_tasks"
sss_topic = "/evolo/sensors/sidescan"

#MQTT bridge parameters
ros_publish_topic = "not_used"
ros_subscribe_topic_1 = status_publish_topic
mqtt_publish_topic_1 =  "e/from/3843318481/sci_status"
ros_subscribe_topic_2 = sidescan_publish_topic
mqtt_publish_topic_2 =  "e/from/3843318481/sidescan"



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
    
    mqtt_bridge_node1 = Node(
        package='evolo_mqtt_bridge',
        namespace=robot_ns,
        executable='bridge',
        name="evolo_node_red_mqtt_1",
        parameters=[{'use_sim_time': use_sim_time,
                     "broker_addr" : broker_addr,
                    "broker_port" : broker_port,
                    "broker_uname" : broker_uname,
                    "broker_pw" : broker_pw,
                    "ros_publish_topic" : ros_publish_topic,
                    "ros_subscribe_topic" : ros_subscribe_topic_1,
                    "mqtt_publish_topic" : mqtt_publish_topic_1
                    #"mqtt_subscribe_topics" : []
                    }])
    
    mqtt_bridge_node2 = Node(
        package='evolo_mqtt_bridge',
        namespace=robot_ns,
        executable='bridge',
        name="evolo_node_red_mqtt_2",
        parameters=[{'use_sim_time': use_sim_time,
                     "broker_addr" : broker_addr,
                    "broker_port" : broker_port,
                    "broker_uname" : broker_uname,
                    "broker_pw" : broker_pw,
                    "ros_publish_topic" : ros_publish_topic,
                    "ros_subscribe_topic" : ros_subscribe_topic_2,
                    "mqtt_publish_topic" : mqtt_publish_topic_2
                    #"mqtt_subscribe_topics" : []
                    }])


    translator_node = Node(package='evolo_node_red_interface',
                            namespace=robot_ns,
                            executable='translator',
                            name='node_red_translator',
                            output='screen',
                            parameters=[{
                                'status_publish_topic': status_publish_topic,
                                'sidescan_publish_topic': sidescan_publish_topic,
                                'sbg_ekf_nav_topic': sbg_ekf_nav_topic,
                                'BT_executing_tasks': BT_executing_tasks,
                                'sss_topic': sss_topic
                            }])


    return LaunchDescription([
        robot_ns_launch_arg,
        sim_time_arg, 
        mqtt_bridge_node1,
        mqtt_bridge_node2,
        translator_node
    ])
