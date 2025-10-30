from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_based_nav',
            executable='rrt_planner',
            output='screen'
        )
    ])
