from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from smarc_msgs.msg import Topics as SmarcTopics
from evolo_msgs.msg import Topics as EvoloTopics


def generate_launch_description():
    # ------- Set up your topic names here. -----------
    cmd_topic = EvoloTopics.EVOLO_ROS_GIMBAL_CAMCMD  # Input high-level CamCmd topic.
    gimbal_ctrl_topic = EvoloTopics.EVOLO_GIMBAL_CTRL  # Output low-level ctrl topic.
    gimbal_feedback_topic = EvoloTopics.EVOLO_GIMBAL_FEEDBACK  # Feedback from gimbal/camera topic.
    odom_topic = SmarcTopics.ODOM_TOPIC  # Odometry topic (orientation).
    geopoint_topic = SmarcTopics.POS_LATLON_TOPIC  # Global position topic.
    # -------------------------------------------------------

    # Namespace as command line argument.
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the nodes.")
    robot_ns = LaunchConfiguration("namespace")
    gimbal_ns = [LaunchConfiguration("namespace"), "/gimbal/"]

    # Whether to use the vehicles altitude or constrain it to the 2d plane.
    altitude_arg = DeclareLaunchArgument(
        "use_vehicle_altitude", default_value='False', description="2D or 3D odom.")
    use_altitude = LaunchConfiguration("use_vehicle_altitude")

    pkg_share = get_package_share_directory("z1_pro_driver")
    urdf_path = os.path.join(pkg_share, "urdf", "z1_pro_camera.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        namespace_arg, altitude_arg,
        # Launch robot state publisher and gimbal joint publisher,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=gimbal_ns,
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": gimbal_ns
            }],
        ),
        Node(
            package="z1_pro_driver",
            executable="gimbal_joint_publisher",
            namespace=gimbal_ns,
            output="screen",
            parameters=[{
                "gimbal_feedback_topic": gimbal_feedback_topic,
                "use_vehicle_altitude": use_altitude
            }],
        ),

        # Launch low-level driver and high-level interface.
        Node(
            package="z1_pro_driver",
            executable="read_and_publish.py",
            namespace=robot_ns,
            output="screen",
            parameters=[{
                "gimbal_ctrl_topic": gimbal_ctrl_topic,
                "gimbal_feedback_topic": gimbal_feedback_topic,
            }],
        ),
        Node(
            package="z1_pro_driver",
            executable="gimbal_interface_node",
            namespace=robot_ns,
            output="screen",
            parameters=[{
                "cmd_topic": cmd_topic,
                "gimbal_ctrl_topic": gimbal_ctrl_topic,
                "odom_topic": odom_topic,
                "geopoint_topic": geopoint_topic
            }],
        ),
    ])
