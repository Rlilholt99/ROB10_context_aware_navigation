from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    semantic_map_server = Node(
        package='context_aware_nav_mapping',
        executable='semantic_map_server',
        name='semantic_map_server'
    )

    return LaunchDescription([
        semantic_map_server
    ])