import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('group10_final'),
        'config',
        'way_points.yaml'
        )
        
    node=Node(
        package = 'group10_final',
        name = 'aruco_subscriber',
        executable = 'aruco_subs',
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
