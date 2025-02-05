from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Waypoint Controller Node
        Node(
            package='waypoint',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen'
        ),
        # Status Monitor Node
        Node(
            package='waypoint',
            executable='status_monitor',
            name='status_monitor',
            output='screen'
        )
    ])

