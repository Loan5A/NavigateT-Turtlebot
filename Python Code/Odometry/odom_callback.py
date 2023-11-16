#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
import threading

from geometry_msgs.msg import Quaternion


class odom_callback(Node):
    def __init__(self):
        super().__init__('odom_callback')

        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.start_time = time.time()



    
    def odometry_callback(self, msg):
        
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        q = orientation
        angles = self.quaternion2euler(q)
        self.theta = angles[-1]

        # Display position and speed
        self.get_logger().info(f"Position - X: {self.x}, Y: {self.y}")
        self.get_logger().info(f"Theta: {self.theta}")


    def quaternion2euler(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        angles = np.array([roll, pitch, yaw]) #yaw is the angle that we want
        return angles
    

def main(args=None):
    rclpy.init(args=args)
    node = odom_callback()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

