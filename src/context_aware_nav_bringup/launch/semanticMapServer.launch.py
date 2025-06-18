from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    context_aware_bringup =  os.path.join(
        get_package_share_directory('context_aware_nav_bringup'),
        'launch')

    start_sim =  os.path.join(
        context_aware_bringup,
        'startSim.launch.py'
    )
    object_detection =  os.path.join(
        context_aware_bringup,
        'objectDetection.launch.py'
    )

    owl_lookup =  os.path.join(
        context_aware_bringup,
        'owlLookup.launch.py'
    )

    nlp_node =  os.path.join(
        context_aware_bringup,
        'nlpNode.launch.py'
    )
    nlp = IncludeLaunchDescription(
        nlp_node
    )

    sim = IncludeLaunchDescription(
        start_sim
    )
    owl = IncludeLaunchDescription(
        owl_lookup
    )
    detection = IncludeLaunchDescription(
        object_detection
    )
    semantic_map_server = Node(
        package='context_aware_nav_mapping',
        executable='semantic_map_server',
        name='semantic_map_server'
    )

    return LaunchDescription([
        sim,
        semantic_map_server,
        owl,
        detection,
        nlp
    ])