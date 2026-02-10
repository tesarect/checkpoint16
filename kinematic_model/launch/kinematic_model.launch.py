from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start wheel_velocities_publisher node
        Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            name='wheel_velocities_publisher',
            output='screen'
        ),
        
        # Start kinematic_model node
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen'
        )
    ])