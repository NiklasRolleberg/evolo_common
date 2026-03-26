from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from evolo_msgs.msg import Topics as evoloTopics


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument('robot_name',
                                                default_value='')

    camcmd_publish_topic = evoloTopics.EVOLO_ROS_GIMBAL_CAMCMD
    string_subscribe_topic = evoloTopics.EVOLO_MQTT_GIMBAL_CAMCMD

    json_bridge_node = Node(package='evolo_gimbal_remote_control',
                            namespace=robot_ns,
                            executable='gimbal_camera_remote_node',
                            name='gimbal_camera_remote_node',
                            output='screen',
                            parameters=[{
                                'string_input_topic':
                                string_subscribe_topic,
                                'camcmd_output_topic':
                                camcmd_publish_topic,
                            }])

    return LaunchDescription([
        robot_ns_launch_arg,
        json_bridge_node,
    ])
