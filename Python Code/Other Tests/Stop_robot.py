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


class MoveController(Node):
    def __init__(self):
        super().__init__('move_controller')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_msg = Twist()
        self.start_time = time.time()
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)

        
def main(args=None):
    rclpy.init(args=args)
    node = MoveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        pass

    

if __name__ == '__main__':
    main()
