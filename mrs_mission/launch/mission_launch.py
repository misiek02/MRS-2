from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('mrs_mission'),
        'config'
    )

    return LaunchDescription([
        Node(
            package='mrs_mission',
            executable='auctioneer_node',
            name='auctioneer_node', # default name. only used if the node did not set its own name in the code (class initialisation)
            parameters=[
                os.path.join(config_dir, 'global_params.yaml'),
                os.path.join(config_dir, 'auctioneer_params.yaml')
            ],
            output='screen'
        ),
        Node(
            package='mrs_mission',
            executable='formation_action',
            name='formation_action_node', # default name. only used if the node did not set its own name in the code (class initialisation)
            parameters=[
                os.path.join(config_dir, 'global_params.yaml')
            ],
            output='screen'
        ),
        Node(
            package='mrs_mission',
            executable='bidder_2',
            name='bidder_2_node', # default name. only used if the node did not set its own name in the code (class initialisation)
            parameters=[
                os.path.join(config_dir, 'global_params.yaml')
            ],
            output='screen'
        ),
        Node(
            package='mrs_mission',
            executable='bidder_3',
            name='bidder_3_node', # default name. only used if the node did not set its own name in the code (class initialisation)
            parameters=[
                os.path.join(config_dir, 'global_params.yaml')
            ],
            output='screen'
        ),
        Node(
            package='mrs_mission',
            executable='bidder_4',
            name='bidder_4_node', # default name. only used if the node did not set its own name in the code (class initialisation)
            parameters=[
                os.path.join(config_dir, 'global_params.yaml')
            ],
            output='screen'
        )
    ])