#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import os

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.trajectory = []
        self.waypoints = [
            (0, 0), (1, -1), (2, 0), (3, 1), 
            (4, 0), (3, -1), (2, 0), (1, 1), (0, 0)
        ]
        
        self.save_path = '/home/user/ros2_ws/src/checkpoint16/eight_trajectory/scripts/'
        
        # Save waypoints immediately
        np.save(self.save_path + 'waypoints.npy', np.array(self.waypoints))
        self.get_logger().info('Waypoints saved')
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.trajectory.append((x, y))
        
    def __del__(self):
        # Save trajectory when node shuts down
        if len(self.trajectory) > 0:
            np.save(self.save_path + 'trajectory.npy', np.array(self.trajectory))
            self.get_logger().info(f'Trajectory saved with {len(self.trajectory)} points')

def main(args=None):
    rclpy.init(args=args)
    logger = TrajectoryLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        # Save trajectory before shutdown
        if len(logger.trajectory) > 0:
            np.save(logger.save_path + 'trajectory.npy', np.array(logger.trajectory))
            logger.get_logger().info(f'Trajectory saved with {len(logger.trajectory)} points')
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()