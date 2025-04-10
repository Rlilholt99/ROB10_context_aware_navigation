from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import os


from ament_index_python.packages import get_package_share_directory







def generate_launch_description():

    spawn_tiago_path =  os.path.join(
        get_package_share_directory('tiago_gazebo'),
        'launch',
        'tiago_gazebo.launch.py'
    )

    spawn_tiago = IncludeLaunchDescription(
        spawn_tiago_path,
        launch_arguments={'is_public_sim':'True',
                          'navigation':'True',
                          'world_name':'contextsim'}.items()
    )
    return LaunchDescription([
        spawn_tiago
    ])