#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from mocap_msgs.msg import RigidBodies
import numpy as np
import math
import time
import threading
import sys
from geometry_msgs.msg import Quaternion


class rigid_body_callback(Node):
    def __init__(self):
        super().__init__('rigid_bodies_callback')

        self.subscription = self.create_subscription(RigidBodies, sys.argv[1] + 'rigid_bodies', self.rigid_bodies_callback, 10)
        self.start_time = time.time()



    
    def rigid_bodies_callback(self, msg):
        # print(f"{msg}")
        x=msg.rigidbodies[6].pose.position.x
        y=msg.rigidbodies[6].pose.position.y
        z=msg.rigidbodies[6].pose.position.z


        qx = msg.rigidbodies[6].pose.orientation.x
        qy = msg.rigidbodies[6].pose.orientation.y
        qz = msg.rigidbodies[6].pose.orientation.z
        qw = msg.rigidbodies[6].pose.orientation.w


        self.get_logger().info(f"x = {x}")
        self.get_logger().info(f"y = {y}")
        self.get_logger().info(f"z = {z}")

        # self.get_logger().info(f"qx = {qx}")
        # self.get_logger().info(f"qy = {qy}")
        # self.get_logger().info(f"qz = {qz}")
        # self.get_logger().info(f"qw = {qw}")

        angles = self.quaternion2euler_qualisys(qx, qy, qz, qw)
        theta = angles[-1]%(2*math.pi)

        # Display position and speed
        self.get_logger().info(f"Theta = {theta}\n")


    def quaternion2euler_qualisys(self, qx, qy, qz, qw):
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (qw * qy - qx * qz))
        cosp = math.sqrt(1 - 2 * (qw * qy - qx * qz))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        angles = np.array([roll, pitch, yaw]) #yaw is the angle that we want
        return angles
    

def main(args=None):
    rclpy.init(args=args)
    node = rigid_body_callback()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

