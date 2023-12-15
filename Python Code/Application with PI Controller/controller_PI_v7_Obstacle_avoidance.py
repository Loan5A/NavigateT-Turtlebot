#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# from mocap_msgs.msg import RigidBodies
import numpy as np
import math
import time
import threading
import pandas as pd
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt
import matplotlib
import sys
import subprocess
import argparse

from rclpy.qos import qos_profile_sensor_data

class MoveController(Node):
    #####################################################################################
    #                                Initialization                                     #
    #####################################################################################
    def __init__(self):
        super().__init__('move_controller')

        # Parsing input arguments
        parser = argparse.ArgumentParser(description="Slave Script")
        parser.add_argument("--tracking_type", type=str, help="Tracking type: 0 = setpoint tracking; 1 = reference tracking")
        parser.add_argument("--x", type=float, help="x for setpoint tracking")
        parser.add_argument("--y", type=float, help="y for setpoint tracking")
        parser.add_argument("--theta", type=str, help="theta for setpoint tracking")
        parser.add_argument("--reach_startpoint", type=float, help="Reach stat point before tracking")
        parser.add_argument("--file_path",type=str, help="Optional file name")
        parser.add_argument("--topic_prefix",type=str, help="Optional file name")

        args = parser.parse_args()

        self.topic_prefix = ""
        if args.topic_prefix:
            self.topic_prefix = args.topic_prefix
            print(self.topic_prefix)

        # Set the flag to switch between odonm and rigid_bodies (Qualisys)
        self.using_odom = True # True = using rigid_bodies

        if self.using_odom:
            self.subscription = self.create_subscription(Odometry, self.topic_prefix + '/odom', self.odometry_callback, qos_profile = qos_profile_sensor_data)
        # else:
        #     self.subscription = self.create_subscription(RigidBodies, 'rigid_bodies', self.rigid_bodies_callback, 10)
        
        self.sub = self.create_subscription(LaserScan, self.topic_prefix + '/scan', self.laser_callback, qos_profile = qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, self.topic_prefix + '/cmd_vel', 10)
        self.cmd_msg = Twist()
        self.start_time = time.time()
        self.end_time = None

        # File to write data
        # self.file = open('odom_data.txt', 'w')

        # Controller constants
        self.kp_distance = 0.9  # Proportional gain for distance control
        self.kp_angle = 0.9  # Proportional gain for angle control
        self.ki_distance = 0.0
        self.ki_angle = 0.0
        
        self.kp_angle_init = 1.5 # Proportionnal gain for start or final angle reaching
        
        #Initialize Integral terms
        self.integral_distance = 0
        self.integral_angle = 0

        # Limits
        self.coef_sat = 0.93
        self.MAX_LINEAR_SPEED = 0.22 * self.coef_sat
        self.MIN_LINEAR_SPEED = -0.22 * self.coef_sat
        self.MAX_ROTATION_SPEED = 2.82 * self.coef_sat
        self.MIN_ROTATION_SPEED = -2.82 * self.coef_sat

        # Robot's position updated by odometry_callback or rigid_bodies_callback
        self.x = None
        self.y = None
        self.z = None
        self.theta = None

        # Memory stamp
        self.x_prev = None
        self.y_prev = None
        self.theta_prev = None

        # Safety distances
        self.safety_radius = 0.18 # safety region = circle of radius 18cm around the turtlebot = 10 cm between the edges and the safety circle
        self.obstacle_max_distance = 0.5 # Turtlebot starts detecting obstacle at 50 cm and less

        # Initialize obstacles
        self.obstacle_angle = None
        self.obstacle_distance = None
        self._180 = 0

        # Initialize vectors for plot and data saving
        self.xplot = []
        self.yplot = []
        self.error = []
        self.time_ = []
        self.distance = 0
        self.speeds = []

        # Constants for tracking options
        SETPOINT_TRACKING = "0"
        REFERENCE_TRACKING = "1"
        START_POINT_REACH = "1"

        self.reachStartPoint = False
        self.tracking_type = None


        # Retrieving tracking type and setpoint from the App
        if args.tracking_type:
            self.tracking_type = args.tracking_type
            if self.tracking_type == SETPOINT_TRACKING:
                self.xref = [float(args.x)]
                self.yref = [float(args.y)]
                self.theta_ref = float(args.theta)
                print(f"Type: Setpoint tracking -> x={self.xref} y={self.yref} theta={self.theta_ref}")
                # time.sleep(5)

            elif self.tracking_type == REFERENCE_TRACKING:
                self.file_path = args.file_path
                self.xref, self.yref = self.importFromCSV(self.file_path)
                self.reachStartPoint = args.reach_startpoint
                print(f"Type: Reference tracking -> File: {self.file_path}, {self.reachStartPoint}")
                # time.sleep(5)

        else:
            print("No tracking type specified. Tracking (0;0) by default")
            self.xref = [0.0]
            self.yref = [0.0]
            self.theta_ref = 0.0
            # time.sleep(5)


        # Variables to update the path
        self.iref = 0
        self.imax = len(self.xref) - 1

        # Parameter for displaying numbers
        self.significant_numbers = 6

        # Sampling time
        self.Ts = 0.1



    #####################################################################################
    #                                   Controller                                      #
    #####################################################################################

    def controller(self):
        start_time = time.time()
        while (self.x is None) or (self.y is None) or (self.theta is None) or (self.obstacle_distance is None):
            elapsed_time = time.time() - start_time
            self.get_logger().warn(f"Waiting for ODOMETRY/RigidBodies for initial position")

            self.get_logger().warn(f"Elapsed Time: {round(elapsed_time, 2)} seconds")
            time.sleep(self.Ts)
            print('\033[F\033[K', end='\r')
            print('\033[F\033[K', end='\r')
            

        # Initiallizing start position
        x = self.x
        self.x_prev = x
        y = self.y
        self.y_prev=y
        theta = self.theta

        module_target = math.sqrt((x - self.xref[0]) ** 2 + (y - self.yref[0]) ** 2)
        argument_target = math.atan2(self.yref[0] - y, self.xref[0] - x)

        time_prev = time.time()
        


        #-------------- First orient the robot in the direction of the start point --------------

        if self.tracking_type == "0" or self.reachStartPoint:
            # Preparing for start angle reaching
            unwrapped_angles = np.unwrap([argument_target, self.theta])
            argument_target = unwrapped_angles[0]
            self.theta = unwrapped_angles[1]

            while abs(argument_target-self.theta)>0.01:
                # Proportional command
                command_angular_speed = self.kp_angle_init* (argument_target-self.theta)

                # Saturation
                command_angular_speed = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)
                
                # Publishing the command
                self.cmd_msg.angular.z = float(command_angular_speed)
                self.publisher.publish(self.cmd_msg)

                # Updating 
                unwrapped_angles = np.unwrap([argument_target, self.theta])
                argument_target = unwrapped_angles[0]
                self.theta = unwrapped_angles[1]

                # Printing infos
                print(f"Reference angle: {argument_target}")
                print(f"Current angle: {self.theta}")
                print(f"Error: {argument_target-self.theta}\n")
                print(f"Command: {command_angular_speed}\n")

            self.get_logger().info(f"Start angle reached !")

            # Stop robot
            self.cmd_msg.angular.z = 0.0
            self.cmd_msg.linear.x = 0.0
            self.publisher.publish(self.cmd_msg)



        #-------------- Then reach the start point --------------

        if self.reachStartPoint:
            # Launch controller as setpoint tracker to track the start point of the trajectory.
            subprocess.run(["python3", "controller_PI_v3.py", "0",str(self.xref[0]), str(self.yref[0]), str((math.atan2(self.yref[1]-self.yref[0],self.xref[1]-self.xref[0])%(2*math.pi)))])



        #-------------- And finally follow the trajectory/Reach the setpoint --------------

        # Starts setpoint or reference tracking
        while self.iref < self.imax or module_target > 0.01:
            # Updating time
            current_time = time.time()

            # Verifying obstacle presence
            if self.obstacle_distance != -1:
                obstacle_angle = self.obstacle_angle
                obstacle_distance = self.obstacle_distance
                safety_margin = obstacle_distance*math.sin(math.radians(obstacle_angle))

                print(f"margin:{safety_margin}, radius:{self.safety_radius}")
                if abs(safety_margin)<self.safety_radius:
                    # If the robot continues forward it will collide the obstacle
                    print(f"Obstacle detected: EMERGENCY STOP")
                    
                    obstacles = self.obstacles
                    obstacle = self.find_obstacle(obstacles, obstacle_angle)
                    print(f"Obstacle found: {obstacle}")
                    self.emergency_controller( obstacle)
                    #break # Break the controller and stop the robot
            
            # Computing tracking error
            eps_x = self.xref[self.iref] - self.x
            eps_y = self.yref[self.iref] - self.y

            # Computing module and argument to get to the target
            module_target = math.sqrt(eps_x ** 2 + eps_y ** 2)
            argument_target = math.atan2(eps_y, eps_x)

            # Wrapping argument to 2pi
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

            # Sleep to ensure sampling time
            time_after_iteration = time.time()
            if time_after_iteration - current_time<self.Ts:
                time.sleep(self.Ts-(time_after_iteration - current_time))
            else:
                time.sleep(self.Ts/10)

        self.end_time = time.time()
        self.get_logger().info(f"Setpoint reached !")

        # Stop robot
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)

        # If we are in setpoint tracking mode, reach the final given angle.
        if self.tracking_type == "0":
            # Preparing for final angle reaching
            print(f"{self.theta_ref}   {self.theta}")
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

        # Adjusting vectors to plot
        while len(self.xref) < len(self.xplot):
            self.xref = np.concatenate([self.xref, [self.xref[-1]]])
        while len(self.yref) < len(self.yplot):
            self.yref = np.concatenate([self.yref, [self.yref[-1]]])

        self.tracking_finished = True
        self.get_logger().info(f"Controller succesfully ended")



    # Emergency controller

    def emergency_controller(self, obstacle):
        # Stop robot
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)

        safety_margins = [obstacle[2]*math.sin(math.radians(obstacle[0])), obstacle[3]*math.sin(math.radians(obstacle[1]))]
        print(f"{safety_margins}")

        if np.mean(safety_margins)>0.0:
            ref = + self.safety_radius
        else:
            ref = - self.safety_radius

        # Deviate only the orientation
        while any(abs(value) < self.safety_radius for value in safety_margins):
            start_time = time.time()

            # Computing command
            command_angular_speed = -5*(ref - min(safety_margins, key=abs))
            print(f"Min: {min(safety_margins, key=abs)}, Cmd: {ref-min(safety_margins)}")

            # Saturation
            command_angular_speed = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)

            # Command publication
            self.cmd_msg.angular.z = command_angular_speed
            self.publisher.publish(self.cmd_msg)

            # Computation of the safety margin
            # safety_margin = self.obstacle_distance*math.sin(math.radians(self.obstacle_angle))
            obstacles = self.obstacles
            obstacle_angle = self.obstacle_angle
            obstacle = self.find_obstacle(obstacles, obstacle_angle)
            safety_margins = [obstacle[2]*math.sin(math.radians(obstacle[0])), obstacle[3]*math.sin(math.radians(obstacle[1]))]
            print(f"SM: {safety_margins}")

            end_time = time.time()
            if end_time - start_time<self.Ts:
                time.sleep(self.Ts-(end_time - start_time))
            else:
                time.sleep(self.Ts/10)

        # Stop robot
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)

        # Move forward to pass the obstacle
        self.cmd_msg.linear.x = 0.15
        self.publisher.publish(self.cmd_msg)

        obstacles = self.obstacles
        obstacle_angle = self.obstacle_angle
        obstacle = self.find_obstacle(obstacles, obstacle_angle)

        # Il faut d'abord vérifier à quel obstacle appartiennent self.obstacle_angle puis vérifier que cet obstacle est bien entièrement entre 90° et 270°
        # while(not(self.obstacle_angle>90 and self.obstacle_angle<270)):
        while not (90 <= obstacle[0] <= 270) or not (90 <= obstacle[1] <= 270):
            start_time = time.time()

            print(f"Depassement d'obstacle")
            self.publisher.publish(self.cmd_msg)

            #print(f"Distance: {self.obstacle_distance}, angle = {self.obstacle_angle}")
            obstacles = self.obstacles
            obstacle_angle = self.obstacle_angle
            obstacle = self.find_obstacle(obstacles, obstacle_angle)
            end_time = time.time()
            if end_time - start_time<self.Ts:
                time.sleep(self.Ts-(end_time - start_time))
            else:
                time.sleep(self.Ts/10)
        print(f"Obstacle depasse")
        # Stop robot
        self.cmd_msg.angular.z = 0.0
        self.cmd_msg.linear.x = 0.0
        self.publisher.publish(self.cmd_msg)



        

    
    #####################################################################################
    #                                   Callbacks                                       #
    #####################################################################################
    
    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        q = orientation
        qw = q.w
        qx = q.x
        qy = q.y
        qz = q.z
        angles = self.quaternion2euler(qx, qy, qz, qw)
        theta = angles[-1]

        # Wrapping theta to 2pi
        self.theta = theta%(2*math.pi)
        self.x = position.x
        self.y = position.y

        # Writing data in file
        # self.file.write(f"Position: ({self.x},{self.y})\nQuaternion: ({q.w,q.x,q.y,q.z})\nTheta: {self.theta}\n\n")


    def rigid_bodies_callback(self, msg):
            # print(f"{msg}")
            x=msg.rigidbodies[6].pose.position.x
            y=msg.rigidbodies[6].pose.position.y
            z=msg.rigidbodies[6].pose.position.z

            qx = msg.rigidbodies[6].pose.orientation.x
            qy = msg.rigidbodies[6].pose.orientation.y
            qz = msg.rigidbodies[6].pose.orientation.z
            qw = msg.rigidbodies[6].pose.orientation.w


            # self.get_logger().info(f"x = {x}")
            # self.get_logger().info(f"y = {y}")
            # self.get_logger().info(f"z = {z}")
            # self.get_logger().info(f"Theta = {theta}")

            # self.get_logger().info(f"qx = {qx}")
            # self.get_logger().info(f"qy = {qy}")
            # self.get_logger().info(f"qz = {qz}")
            # self.get_logger().info(f"qw = {qw}\n")

            angles = self.quaternion2euler(qx, qy, qz, qw)
            self.theta = angles[-1]%(2*math.pi)
            self.x = x
            self.y = y


    # Gets the angle and the distance of an obstacle in the front plan, if the obstacle is self.obstacle_max_distance far away or less
    # Notice that when no obstacle is detected, we set self.obstacle_distance to -1
    def laser_callback(self, msg):
        
        # First save the nearest obstacle's point
        whole_region = msg.ranges
        front_region = whole_region[:90] + whole_region[-90:]
        nb_samples = len(whole_region)
        sampling_step = nb_samples/360 # [°]
        
        
        # Getting the distance to the obstacle and its angle (beeing the index)
        has_numeric_value = np.any(np.isfinite(front_region))

        if has_numeric_value:
            index, obstacle_distance = min(enumerate(whole_region), key=lambda x: x[1])
            obstacle_angle = ((index*360/nb_samples)-self._180)%(360)
            self.obstacle_angle = obstacle_angle
            if (obstacle_distance<self.obstacle_max_distance) and (obstacle_angle<=90 or obstacle_angle>=270):
                self.obstacle_distance = obstacle_distance
            else:
                self.obstacle_distance = -1
        else:
            # If there is no obstacle, we set distance to -1
            self.obstacle_distance = -1
        #print(f"Distance: {self.obstacle_distance}, angle = {self.obstacle_angle}")


        #Then, we make the list of all obstacles
        # List of obstacles
        obstacles = []
        obstacle_start_angle = None
        for i in range(len(whole_region)):
            current_distance = whole_region[i]

            # Verify if the begining of an obstacle was found
            if current_distance <= self.obstacle_max_distance and obstacle_start_angle is None:
                obstacle_start_index = (i)%(nb_samples) # Getting a margin of a sampling_step
                obstacle_start_angle = (round((obstacle_start_index)*360/nb_samples-self._180))%360 

            # If the beginning of an obstacle was found and we don't detect anything anymore,
            # then the i-1 angle was the end of the obstacle
            elif current_distance > self.obstacle_max_distance and obstacle_start_angle is not None:
                obstacle_end_index = i - 1 # Save the previous angle but take a margin so i-1+1=i
                obstacle_end_angle = (round((obstacle_end_index)*360/nb_samples-self._180))%360

                # Estimation of the obstacle's radius: Doesn't work because the lidar does not sample enough
                # d1 = whole_region[obstacle_start_angle]
                # theta_0 = round((obstacle_end_angle + obstacle_start_angle)/2)
                # # d0 = whole_region[theta_0]
                # d0 = min(whole_region[obstacle_start_angle:i])

                # r = round(1/2*(d1**2/d0-d0), 3)

                obstacles.append((obstacle_start_angle, obstacle_end_angle, whole_region[obstacle_start_index], whole_region[obstacle_end_index])) # Adding the obstacle to the list
                obstacle_start_angle = None # End of the obstacle, resetting the obstacle detection

        # After the loop, if an obstacle is still detected at the last angle:
        if (whole_region[-1]<=self.obstacle_max_distance) and (obstacle_start_angle is not None):
            # And if the first angle also contains an obstacle
            if (whole_region[0] <= self.obstacle_max_distance):
                obstacles.append((obstacle_start_angle, obstacles[0][1], whole_region[obstacle_start_index], obstacles[0][3])) # append a new obstacle within the discontinuity 360°/0°
                obstacles.pop(0) # Remove the forst obstacle which is actually the second part of this particular obstacle
            # And if the first angle doesn't contain anything, the obstacle finishes at the last angle.
            else:
                obstacle_end_index = nb_samples - 1
                obstacle_start_angle = (round((obstacle_start_index)*360/nb_samples-self._180))%360 
                obstacles.append((obstacle_start_angle, obstacle_end_angle, whole_region[obstacle_start_index], whole_region[obstacle_end_index]))

        self.obstacles = obstacles

            

    #####################################################################################
    #                                 Other functions                                   #
    #####################################################################################

    # Converts quaternions into roll, pitch and yaw angles.
    def quaternion2euler(self, qx, qy, qz, qw):
        # roll (x-axis rotation)
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

        angles = np.array([roll, pitch, yaw]) # Yaw is the angle that we want
        return angles
    
    # Converts path coordinates contained in a CSV file into x and y vectors
    def importFromCSV(self, filename):
        # Read CSV file with data
        df = pd.read_csv(filename, delimiter=';')

        # Extract x and y vectors
        x = df.iloc[:, 0].values  # First column
        y = df.iloc[:, 1].values  # Second column

        # Return x and y vectors
        return x, y


    def is_angle_inside(self, angle, obstacle):
        # Vérifie si l'angle donné se trouve à l'intérieur de l'obstacle
        start_angle = obstacle[0]
        end_angle = obstacle[1]
        if start_angle <= end_angle:
            return start_angle <= angle <= end_angle
        else:
            # Gestion de la discontinuité 360°/0°
            return angle >= start_angle or angle <= end_angle
    
    def find_obstacle(self, obstacles, angle):
        # Retourne l'indice de l'obstacle auquel appartient l'angle donné
        for obstacle in obstacles:
            if self.is_angle_inside(angle, obstacle):
                return obstacle
        return None  # Aucun obstacle trouvé pour l'angle donné

        
    


#####################################################################################
#                                   Main loop                                       #
#####################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = MoveController()

    try:
        # Start controller thread
        controller_thread = threading.Thread(target=node.controller)
        controller_thread.daemon = True  #  Allows the thread to end wen the node is finished
        controller_thread.start()

        node.tracking_finished = False
        while rclpy.ok() and not node.tracking_finished:
            # Spin the node once to process any callbacks
            rclpy.spin_once(node, timeout_sec=0.1)


    except KeyboardInterrupt:
        # Plot result even when interrupting function
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
    
    # Wait for the thread to stop
    node.get_logger().info(f"joining thread")
    controller_thread.join()

    # Plot result
    plt.plot(node.xplot, node.yplot, 'xr')
    plt.plot(node.xref, node.yref, '-b')
    plt.show()

    plt.plot(node.time_, node.xplot, 'xr')
    plt.plot(node.time_[:len(node.xref)], node.xref, '-b')
    plt.plot(node.time_, node.yplot, 'xr')
    plt.plot(node.time_[:len(node.yref)], node.yref, '-b')
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
