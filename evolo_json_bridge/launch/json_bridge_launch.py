from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics


# topics for translator to publish to 
status_publish_topic = "waraps/sensor/scientist"
sidescan_publish_topic = "waraps/sensor/sidescan"

#topics for translator to subscribe to
sbg_ekf_nav_topic = "/evolo/sbg/ekf_nav"
sss_topic = "/evolo/sensors/sidescan"



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
  


    translator_node = Node(package='evolo_json_bridge',
                            namespace=robot_ns,
                            executable='translator',
                            name='evolo_json_bridge',
                            output='screen',
                            parameters=[{
                                'status_publish_topic': status_publish_topic,
                                'sidescan_publish_topic': sidescan_publish_topic,
                                'sbg_ekf_nav_topic': sbg_ekf_nav_topic,
                                'sss_topic': sss_topic
                            }])


    return LaunchDescription([
        robot_ns_launch_arg,
        sim_time_arg, 
        translator_node
    ])
