from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("evolo_config")
    udp_config = config_dir + "/config/udp_evolo.yaml"

    return LaunchDescription(
        [
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="udp_evolo_bridge",
                output="screen",
                parameters=[udp_config],
            ),
        ]
    )
