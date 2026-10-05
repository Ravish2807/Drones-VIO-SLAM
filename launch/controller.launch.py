import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drones_controller')

    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    vehicle_config = os.path.join(pkg_share, 'config', 'vehicle.yaml')

    controller_node = Node(
        package='drones_controller',
        executable='controller_node',
        name='so3_controller',
        output='screen',
        parameters=[
            controller_config,
            vehicle_config
        ]
    )

    return LaunchDescription([
        controller_node
    ])
