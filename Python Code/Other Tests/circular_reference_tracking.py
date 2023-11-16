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
        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_msg = Twist()
        self.start_time = time.time()

        # Controller constants
        self.kp_distance = 0.9  # Proportional gain for distance control
        self.kp_angle = 0.9  # Proportional gain for angle control

        # Limits
        self.MAX_LINEAR_SPEED = 0.22
        self.MIN_LINEAR_SPEED = -0.22
        self.MAX_ROTATION_SPEED = 2.84
        self.MIN_ROTATION_SPEED = -2.84

        # Setpoint
        # self.xref = 0
        # self.yref = 0
        # self.theta_ref = math.pi
        self.T = 60 #Period for the circle
        self.r = 1  #Radius for the circle

        # Robot's position updated by odometry_callback
        self.x = None
        self.y = None
        self.z = None
        self.theta = None

        # Memory stamp
        self.theta_prev = None

        # Flags
        self.initialized = False

        self.control_thread = threading.Thread(target=self.controller)
        self.control_thread.daemon = True  #  Allows the thread to end wen the node is finished
        self.control_thread.start()


    def controller(self):
        while (self.x is None) or (self.y is None) or (self.theta is None):
            self.get_logger().info("Waiting for ODOMETRY to publish its initial position")
        t=0
        xref = math.cos(2*math.pi*t/self.T)
        yref = math.sin(2*math.pi*t/self.T)

        x = self.x
        y = self.y
        theta = self.theta

        module_target = math.sqrt((x - xref) ** 2 + (y - yref) ** 2)
        argument_prev = math.atan2(yref - y, xref - x)
        theta_prev = theta
        
        while module_target > 0.01:
            if t >= self.T:
                t=0
            xref = self.r * math.cos(2*math.pi*t/self.T)
            yref = self.r * math.sin(2*math.pi*t/self.T)

            eps_x = xref - self.x
            eps_y = yref - self.y
            theta = self.theta

            # Computing module and argument to get to the target
            module_target = math.sqrt(eps_x ** 2 + eps_y ** 2)
            argument_target = math.atan2(eps_y, eps_x)

            # Unwrapping argument to avoid discontinuities
            arguments = np.array([argument_prev, argument_target])
            unwrapped_arguments = np.unwrap(arguments)
            argument_target = unwrapped_arguments[-1]  # Access the last element

            command_linear_speed = self.kp_distance * module_target
            command_angular_speed = self.kp_angle * (argument_target - theta)

            # Display distance to setpoint and angle error
            self.get_logger().info(f"Distance : {module_target}     Angle error : {argument_target-theta}")

            # Saturation
            command_linear_speed = min(max(command_linear_speed, self.MIN_LINEAR_SPEED), self.MAX_LINEAR_SPEED)
            command_angular_speed = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)

            self.cmd_msg.angular.z = command_angular_speed
            self.cmd_msg.linear.x = command_linear_speed
            self.publisher.publish(self.cmd_msg)

            argument_prev = argument_target

            t = t + 0.1 # Updating time
            time.sleep(0.1)
        self.get_logger().info("Setpoint reached !")
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)


    def odometry_callback(self, msg):
        
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        q = orientation
        angles = self.quaternion2euler(q)
        theta = angles[-1]

        if (not(self.initialized)):
            self.theta_prev = theta
            self.initialized = True

        # Unwrapping theta to avoid dicontinuities
        angles = np.array([self.theta_prev, theta])
        unwrapped_angles = np.unwrap(angles)
        self.theta = unwrapped_angles[-1]
        self.theta_prev = unwrapped_angles[-1]



        # Display position and speed
        # self.get_logger().info(f"Position - X: {self.x}, Y: {self.y}")
        # self.get_logger().info(f"Theta: {self.theta}")


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

        angles = np.array([roll, pitch, yaw]) # yaw is the angle that we want
        return angles
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
