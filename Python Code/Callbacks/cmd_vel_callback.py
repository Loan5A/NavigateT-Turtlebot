#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class cmd_vel_callback(Node):
    def __init__(self):
        super().__init__('odom_callback')

        self.subscription = self.create_subscription(Twist, sys.argv[1] +'cmd_vel', self.cmd_callback, 10)
        self.start_time = time.time()



    
    def cmd_callback(self, msg):
        
        linear = msg.linear
        angular = msg.angular
        self.linear_x = linear.x
        self.angular_z = angular.z

        # Display speeds
        self.get_logger().info(f"Linear - X: {self.linear_x}")
        self.get_logger().info(f"Angular - Z: {self.angular_z}")
    

def main(args=None):
    rclpy.init(args=args)
    node = cmd_vel_callback()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

