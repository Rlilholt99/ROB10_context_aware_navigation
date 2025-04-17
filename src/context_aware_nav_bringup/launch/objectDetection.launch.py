
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os


from ament_index_python.packages import get_package_share_directory







def generate_launch_description():

    object_detection = Node(
        package='stereo_panoptic_segmentation',
        executable='SPS_node',
        name='object_detection_node',
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        object_detection
    ])