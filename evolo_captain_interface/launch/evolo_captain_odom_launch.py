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

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    mqtt_odom_node = Node(
        package='evolo_captain_to_odom',
        namespace=robot_ns,
        executable='captain_odom',
        name="captain_odom_node",
        parameters=[{'use_sim_time': use_sim_time, 
                     "correct_meridian_convergence" : True,
                     "publish_tf" : True,
                     "output_rate" : 5.0,
                     "verbose_setup" : False,
                     "verbose_conversion" : False,
                     "input_topic" : evoloTopics.EVOLO_CAPTAIN_STATE
                    }]
    )

    return LaunchDescription([
        robot_ns_launch_arg, 
        mqtt_odom_node
    ])
