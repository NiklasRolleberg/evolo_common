import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	config = os.path.join(
		get_package_share_directory('evolo_config'),
		'config',
		'ellipse_D_default.yaml'
	)

	robot_ns = LaunchConfiguration('robot_name')

	robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='evolo'
    )

	sbg_node = Node(
        package='sbg_driver',
		namespace=robot_ns,
		executable = 'sbg_device',
		output = 'screen',
		parameters = [config])

	return LaunchDescription([
		robot_ns_launch_arg,
		sbg_node
	])

