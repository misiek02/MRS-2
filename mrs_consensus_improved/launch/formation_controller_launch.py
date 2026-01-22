from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('mrs_consensus_improved'),
        'config'
    )

    return LaunchDescription([
        Node(
            package='mrs_consensus_improved',
            executable='formation_controller',
            name='formation_controller', # default name. only used if the node did not set its own name in the code (class initialisation)
            parameters=[
                os.path.join(config_dir, 'global_params.yaml'),
                os.path.join(config_dir, 'formation_params.yaml')
            ],
            output='screen'
        )
    ])