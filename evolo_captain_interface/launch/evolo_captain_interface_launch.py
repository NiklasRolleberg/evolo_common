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


    captain_interface = Node(
        package='evolo_captain_interface',
        namespace=robot_ns,
        executable='interface',
        name="evolo_captain_interface",
        parameters=[]
    )


    return LaunchDescription([
        robot_ns_launch_arg, 
        captain_interface
    ])
