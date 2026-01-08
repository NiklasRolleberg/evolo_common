from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo'
    )

    mqtt_odom_init_node = Node(
        package='evolo_captain_to_odom',
        namespace=robot_ns,
        executable='odom_initializer',
        name="captain_odom_initializer",
        parameters=[{"update_rate": 0.1,
                     "verbose": True,
                     "captain_topic" : evoloTopics.EVOLO_CAPTAIN_STATE
                     }]
    )

    return LaunchDescription([
        robot_ns_launch_arg, 
        mqtt_odom_init_node
    ])
