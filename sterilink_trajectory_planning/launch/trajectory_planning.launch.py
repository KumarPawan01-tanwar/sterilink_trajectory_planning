from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sterilink_trajectory_planning',
            executable='trajectory_planning_node',
            name='trajectory_planning_node',
            output='screen'
        )
    ])
