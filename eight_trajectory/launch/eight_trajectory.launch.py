from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rviz_config_path = os.path.join(
        get_package_share_directory('kinematic_model'),
        'real_robot.rviz'
    )

    return LaunchDescription([

        # Rviz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_path]
        # ),

        # Start kinematic_model node to convert wheel speeds to cmd_vel
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen'
        ),
        
        # Start eight_trajectory node
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            name='eight_trajectory',
            output='screen'
        ),

        # Logger (records trajectory)
        Node(
            package='eight_trajectory',
            executable='trajectory_logger.py',
            name='trajectory_logger',
            output='screen'
        ),

        # Node(
        #     package='eight_trajectory',
        #     executable='loop_trajectory.py',
        #     name='loop_trajectory',
        #     output='screen'
        # ),

    ])