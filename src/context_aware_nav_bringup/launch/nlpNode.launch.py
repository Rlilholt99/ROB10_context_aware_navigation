from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nlp_node = Node(
        package='nlp_command_processor',
        executable='nlp_node',
        name='nlp_node'
    )

    return LaunchDescription([
        nlp_node
    ])