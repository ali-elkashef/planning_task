from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='global_planner',
            executable='A_algorithm',
            name='A_algorithm'
        ),
        Node(
            package='global_planner',
            executable='starting_point',
            name='starting_point'
        ),
        Node(
            package='global_planner',
            executable='random_map',
            name='random_map'
        ),
    ])
