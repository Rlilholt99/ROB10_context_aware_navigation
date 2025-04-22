from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    owl_lookup = Node(
        package='owl_graph',
        executable='owl_graph_node',
        name='owl_lookup_node'
    )

    return LaunchDescription([
        owl_lookup
    ])