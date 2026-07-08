from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('arm_controller'),
        'config',
        'arm_controller.yaml'
    )

    arm_controller_node = Node(
        package='arm_controller',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        arm_controller_node
    ])
