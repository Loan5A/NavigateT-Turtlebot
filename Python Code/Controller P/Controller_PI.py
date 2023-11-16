#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
import threading
import pandas as pd
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt


class MoveController(Node):
    def __init__(self):
        super().__init__('move_controller')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_msg = Twist()
        self.start_time = time.time()
        self.end_time = None

        # File to write data
        # self.file = open('odom_data.txt', 'w')

        # Controller constants
        self.kp_distance = 0.9  # Proportional gain for distance control
        self.kp_angle = 0.9  # Proportional gain for angle control
        self.ki_distance = 0 #0.02
        self.ki_angle = 0
        
        self.kp_angle_init = 1.5
        
        #Initialize Integral term
        self.integral_distance = 0
        self.integral_angle = 0.05

        # Limits
        # Max linéaire 0.99*0.22
        # Max angulaire 0.93*2.82
        self.MAX_LINEAR_SPEED = 0.22 * 0.93
        self.MIN_LINEAR_SPEED = -0.22 * 0.93
        self.MAX_ROTATION_SPEED = 2.82 * 0.93
        self.MIN_ROTATION_SPEED = -2.82 * 0.93

        # Robot's position updated by odometry_callback
        self.x = None
        self.y = None
        self.z = None
        self.theta = None

        # Memory stamp
        self.x_prev = None
        self.y_prev = None
        self.theta_prev = None


        # Initialize vectors for plot and data saving
        self.xplot = []
        self.yplot = []
        self.error = []
        self.time_ = []
        self.distance = 0
        self.speeds = []

        # Setpoint
        self.xref = [0] * 200
        self.yref = [0] * 200
        self.theta_ref = 0

        # Initialize path from csv file and counter for updating path
        # self.xref, self.yref = self.importFromCSV('spline.csv')
        self.iref = 1
        self.imax = len(self.xref) - 1

        # Parameter for displaying numbers
        self.significant_numbers = 6

        # Start controller thread
        self.control_thread = threading.Thread(target=self.controller)
        self.control_thread.daemon = True  #  Allows the thread to end when the node is finished
        self.control_thread.start()

        # Sampling time
        self.Ts = 0.05


    def controller(self):
        while (self.x is None) or (self.y is None) or (self.theta is None):
            self.get_logger().info("Waiting for ODOMETRY to publish its initial position")

        # Initiallizing start position
        x = self.x
        self.x_prev = x
        y = self.y
        self.y_prev=y
        theta = self.theta

        module_target = math.sqrt((x - self.xref[1]) ** 2 + (y - self.yref[1]) ** 2)
        argument_target = math.atan2(self.yref[1] - y, self.xref[1] - x)

        time_prev = time.time()
        
        # Preparing for start angle reaching
        unwrapped_angles = np.unwrap([argument_target, self.theta])
        self.theta_ref = unwrapped_angles[0]
        self.theta = unwrapped_angles[1]

        while abs(argument_target-self.theta)>0.01:
            command_angular_speed = self.kp_angle_init * (self.theta_ref-self.theta)
            command_angular_speed = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)
            self.cmd_msg.angular.z = command_angular_speed
            self.publisher.publish(self.cmd_msg)

            unwrapped_angles = np.unwrap([argument_target, self.theta])
            argument_target = unwrapped_angles[0]
            self.theta = unwrapped_angles[1]
            print(f"Reference angle: {self.theta_ref}")
            print(f"Current angle: {self.theta}")
            print(f"Error: {self.theta_ref-self.theta}\n")
            print(f"Command: {command_angular_speed}\n")

        self.get_logger().info(f"Start angle reached !")

        while self.iref < self.imax or module_target > 0.01:
            # Updating time
            current_time = time.time()

            # Computing error
            eps_x = self.xref[self.iref] - self.x
            eps_y = self.yref[self.iref] - self.y

            # Computing module and argument to get to the target
            module_target = math.sqrt(eps_x ** 2 + eps_y ** 2)
            argument_target = math.atan2(eps_y, eps_x)

            # Unwrapping argument to avoid discontinuities

            argument_target = argument_target%(2*math.pi)

            unwrapped_angles = np.unwrap([argument_target, self.theta])
            argument_target = unwrapped_angles[0]
            self.theta = unwrapped_angles[1]

            # Computing Integral term of PI controller
            self.integral_distance = self.integral_distance + self.ki_distance * (current_time - time_prev) * module_target
            self.integral_angle = self.integral_angle + self.ki_angle * (current_time - time_prev) * (argument_target - self.theta)

            # Command signal
            command_linear_speed = self.kp_distance * module_target + self.integral_distance
            command_angular_speed = self.kp_angle * (argument_target - self.theta) + self.integral_angle

            #Display distance to setpoint and angle error
            self.get_logger().info(f"Distance : {round(module_target, self.significant_numbers)}")
            self.get_logger().info(f"Angle error : {round(argument_target-self.theta, self.significant_numbers)}")
            self.get_logger().info(f"Delta t : {round(current_time - time_prev, self.significant_numbers)}")
            self.get_logger().info(f"step : {self.iref}/{self.imax}\n",)

            # Saturation
            command_linear_speed = min(max(command_linear_speed, self.MIN_LINEAR_SPEED), self.MAX_LINEAR_SPEED)
            command_angular_speed = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)

            # Command publication
            self.cmd_msg.angular.z = command_angular_speed
            self.cmd_msg.linear.x = command_linear_speed
            self.publisher.publish(self.cmd_msg)

            # Updating counter
            self.iref = min(self.iref + 1, self.imax) # When the terminal point has arrived, we keep it in time

            # Updating trajectory to plot and datas
            self.xplot.append(self.x)
            self.yplot.append(self.y)
            self.error.append(module_target)
            self.time_.append(current_time)
            segment = np.sqrt((self.x-self.x_prev) ** 2 +(self.y-self.y_prev) ** 2 )
            self.distance = self.distance + segment
            self.speeds.append(segment/(current_time-time_prev))

            # Memory stamp
            argument_prev = argument_target
            time_prev = current_time
            self.x_prev = self.x
            self.y_prev = self.y

            time_after_iteration = time.time()
            time.sleep(self.Ts-(time_after_iteration - current_time))

        self.end_time = time.time()
        self.get_logger().info(f"Setpoint reached !")

        # Stop robot
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)

        # # Preparing for final angle reaching
        unwrapped_angles = np.unwrap([self.theta_ref, self.theta])
        self.theta_ref = unwrapped_angles[0]
        self.theta = unwrapped_angles[1]

        while abs(self.theta_ref-self.theta)>0.01:
            command_angular_speed = 2*(self.theta_ref-self.theta)
            self.cmd_msg.angular.z = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)
            self.publisher.publish(self.cmd_msg)

            unwrapped_angles = np.unwrap([self.theta_ref, self.theta])
            self.theta_ref = unwrapped_angles[0]
            self.theta = unwrapped_angles[1]
            print(f"Reference angle: {self.theta_ref}")
            print(f"Current angle: {self.theta}")
            print(f"Error: {self.theta_ref-self.theta}\n")

        self.get_logger().info(f"Reference angle reached !")
        
        self.get_logger().info(f"\nFinal datas :")
        self.get_logger().info(f"Distance : {round(module_target, self.significant_numbers)}")
        self.get_logger().info(f"Angle error : {round(argument_target-self.theta, self.significant_numbers)}")
        self.get_logger().info(f"Average tracking error : {round(np.mean(self.error), self.significant_numbers)} [m]")
        self.get_logger().info(f"Task achieved in : {round(self.end_time - self.start_time, self.significant_numbers)} [s]")
        self.get_logger().info(f"Distance traveled : {round(self.distance, self.significant_numbers)} [m]")
        self.get_logger().info(f"Aveage speed : {round(np.mean(self.speeds), self.significant_numbers)} [m/s]")

        # Stop robot
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)

        # Plot result
        # plt.plot(self.xplot, self.yplot, 'xr')
        # plt.plot(self.xref, self.yref, '-b')
        # plt.show()
    

    def odometry_callback(self, msg):
        
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        q = orientation
        angles = self.quaternion2euler(q)
        theta = angles[-1]

        # Wrapping theta to 2pi
        self.theta = theta%(2*math.pi)

        # Writing data in file
        # self.file.write(f"Position: ({self.x},{self.y})\nQuaternion: ({q.w,q.x,q.y,q.z})\nTheta: {self.theta}\n\n")


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
    
    def importFromCSV(self, filename):
        # Read CSV file with data
        df = pd.read_csv(filename, delimiter=';')

        # Extract x and y vectors
        x = df.iloc[:, 0].values  # La première colonne
        y = df.iloc[:, 1].values  # La deuxième colonne

        # Return x and y vectors
        return x, y
        
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Plot result
        plt.plot(node.xplot, node.yplot, 'xr')
        plt.plot(node.xref, node.yref, '-b')
        plt.show()

        plt.plot(node.time_, node.xplot, 'xr')
        plt.plot(node.time_[:len(node.xref)], node.xref, '-b')
        plt.plot(node.time_, node.yplot, 'xr')
        plt.plot(node.time_[:len(node.yref)], node.yref, '-b')
        plt.show()

        node.publisher = node.create_publisher(Twist, 'cmd_vel', 10)
        node.cmd_msg = Twist()

        node.cmd_msg.angular.z = 0.0
        node.cmd_msg.linear.x = 0.0
        node.publisher.publish(node.cmd_msg)
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
