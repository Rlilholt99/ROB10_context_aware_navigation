from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os


from ament_index_python.packages import get_package_share_directory







def generate_launch_description():

    semantic_mapping = Node(
        package='context_aware_nav_mapping',
        executable='semantic_mapping',
        name='semantinc_mapping_node'
    )


    return LaunchDescription([
        semantic_mapping
    ])