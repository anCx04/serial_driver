import os
from ament_index_python.packages import get_package_share_directory

#! These are necessary to build the launch description
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('serial_node'),
        'config',
        'serial_parameters.yaml'
    )

    # Create node launch description
    node = Node(
        package='serial_node',
        executable='serial_node',
        name='serial_node',
        parameters=[config] #! We could specify more than one source here
    )

    # Finalize launch description
    ld.add_action(node)
    return ld