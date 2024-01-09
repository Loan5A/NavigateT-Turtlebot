#!/usr/bin/env python3

# ROS
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Topics
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# from mocap_msgs.msg import RigidBodies

#Basic python packages
import numpy as np
import math
import time
import threading
import pandas as pd

# Plots
import matplotlib.pyplot as plt
import matplotlib

# Subprocesses & arguments
import sys
import subprocess
import argparse
import os

# Obstacle management
from classes.obstacle import Obstacle
from classes.chronometer import Chronometer



class MoveController(Node):
    #####################################################################################
    #                                Initialization                                     #
    #####################################################################################
    def __init__(self):
        super().__init__('move_controller')
        ###################################
        #     Definition of constants     #
        ###################################

        # Constants for tracking options
        self.SETPOINT_TRACKING = "0"
        self.REFERENCE_TRACKING = "1"
        self.START_POINT_REACH = "1"

        # Limits of the Turtlebot3: Burger
        self.MAX_LINEAR_SPEED = 0.22
        self.MIN_LINEAR_SPEED = -0.22
        self.MAX_ROTATION_SPEED = 2.82
        self.MIN_ROTATION_SPEED = -2.82

        # Constraints on command variations
        self.LIN_VEL_STEP_SIZE = 0.04
        self.ANG_VEL_STEP_SIZE = 0.4

        # Obstacle avoidance side
        self.LEFT_AVOIDANCE = 1
        self.RIGHT_AVOIDANCE = -1

        # Controller constants
        self.kp_distance = 0.9  # Proportional gain for distance control
        self.kp_angle = 0.9  # Proportional gain for angle control
        self.ki_distance = 0.0
        self.ki_angle = 0.0
        self.kp_angle_init = 1.5 # Proportionnal gain for start or final angle reaching

        # Maximum errors
        self.distance_error = 0.01
        self.angle_error = 0.01

        # Parameter for displaying numbers
        self.significant_numbers = 6

        # Sampling time
        self.Ts = 0.1

        # Safety distances
        self.safety_radius = 0.25 # safety region = circle of radius 18cm around the turtlebot = 10 cm between the edges and the safety circle
        self.obstacle_max_distance = 0.5 # Turtlebot starts detecting obstacle at 50 cm and less



        ###################################
        #        Treating arguments       #
        ###################################

        # Parsing input arguments
        parser = argparse.ArgumentParser(description="Slave Script")
        parser.add_argument("--tracking_type", type=str, help="Tracking type: 0 = setpoint tracking; 1 = reference tracking")
        parser.add_argument("--x", type=float, help="x for setpoint tracking")
        parser.add_argument("--y", type=float, help="y for setpoint tracking")
        parser.add_argument("--theta", type=str, help="theta for setpoint tracking")
        parser.add_argument("--reach_startpoint", type=float, help="Reach stat point before tracking")
        parser.add_argument("--file_path",type=str, help="Optional file name")
        parser.add_argument("--topic_prefix",type=str, help="Optional file name")
        parser.add_argument("--plots", type=float, help="set or not plots")
        parser.add_argument("--obstacle_avoidance", type=float, help="set or not obstacle avoidance")
        parser.add_argument("--real", type=float, help="add 180 °")
        parser.add_argument("--odometry", type=float, help="odom or rigid bodies")

        args = parser.parse_args()

        self.reachStartPoint = False
        self.tracking_type = None

        self.topic_prefix = ""
        if args.topic_prefix:
            self.topic_prefix = args.topic_prefix
            print(self.topic_prefix)

        # Retrieving tracking type and setpoint from the App
        if args.tracking_type:
            self.tracking_type = args.tracking_type
            if self.tracking_type == self.SETPOINT_TRACKING:
                self.xref = [float(args.x)]
                self.yref = [float(args.y)]
                self.theta_ref = float(args.theta)
                print(f"Type: Setpoint tracking -> x={self.xref} y={self.yref} theta={self.theta_ref}")

            elif self.tracking_type == self.REFERENCE_TRACKING:
                self.file_path = args.file_path
                self.xref, self.yref = self.importFromCSV(self.file_path)
                self.reachStartPoint = args.reach_startpoint
                print(f"Type: Reference tracking -> File: {self.file_path}, {self.reachStartPoint}")

        else:
            print("No tracking type specified. Tracking (0;0) by default")
            self.xref = [0.0]
            self.yref = [0.0]
            self.theta_ref = 0.0

        #Retrieving boolean to set or not plots and obstacle avoidance
        self.setPlots = bool(args.plots)
        self.SetObstacleAvoidance = bool(args.obstacle_avoidance)
        self.SetReal = bool(args.real)
        self.SetOdom = bool(args.odometry)

        ###################################
        #     Subscriptions/Publishers    #
        ###################################
            
        # Set the flag to switch between odonm and rigid_bodies (Qualisys)
        self.using_odom = True # False = using rigid_bodies

        if self.SetOdom:
            self.subscription = self.create_subscription(Odometry, self.topic_prefix + '/odom', self.odometry_callback, qos_profile = qos_profile_sensor_data)
        # else:
        #     self.subscription = self.create_subscription(RigidBodies, 'rigid_bodies', self.rigid_bodies_callback, 10)
        
        self.sub = self.create_subscription(LaserScan, self.topic_prefix + '/scan', self.laser_callback, qos_profile = qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, self.topic_prefix + '/cmd_vel', 10)
        self.cmd_msg = Twist()



        #########################################
        #    Initialization of work variables   #
        #########################################

        # Time variables
        self.start_time = time.time()
        self.end_time = None

        #Initialize Integral terms
        self.integral_distance = 0
        self.integral_angle = 0

        # Variables to store commands
        self.command_linear_speed = 0.00
        self.command_angular_speed = 0.00

        # Robot's position updated by odometry_callback or rigid_bodies_callback
        self.x = None
        self.y = None
        self.z = None
        self.theta = None

        # Memory stamp
        self.x_prev = None
        self.y_prev = None
        self.theta_prev = None

        # Initialize obstacles
        self.obstacle_angle = None
        self.obstacle_distance = None

        if self.SetReal:
            self._180 = 180 # Set it to 180 for real application and 0 for simulation
        else:
            self._180 = 0

        # Variable for obstacle avoidance state
        self.obstacle_avoidance = None

        # Initialize vectors for plot and data saving
        self.xplot = []
        self.yplot = []
        self.error = []
        self.time_ = []
        self.distance = 0
        self.speeds = []

        self._xplot = []
        self._yplot = []
        self._distance = 0
        self._speeds = []
        self._accelerations = []
        self._time = []
        self._xcarthography = []
        self._ycarthography = []

        # Variables to update the path
        self.iref = 0
        self.imax = len(self.xref) - 1

        # Initiallizing chronometers
        self.loop_chrono = Chronometer(sampling_time = self.Ts)
        self.task_chrono = Chronometer(sampling_time = self.Ts)

        # Boolean for end of track
        self.tracking_finished = False



    #####################################################################################
    #                                   Controller                                      #
    #####################################################################################

    def controller(self):
        self.task_chrono.start()
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

        if self.tracking_type == self.SETPOINT_TRACKING or self.reachStartPoint:
            # Preparing for start angle reaching
            unwrapped_angles = np.unwrap([argument_target, self.theta])
            argument_target = unwrapped_angles[0]
            theta = unwrapped_angles[1]

            while abs(argument_target-theta) > self.angle_error:
                # Saving time
                self.loop_chrono.start()

                # Updating 
                argument_target = math.atan2(self.yref[0] - self.y, self.xref[0] - self.x)
                unwrapped_angles = np.unwrap([argument_target, self.theta])
                argument_target = unwrapped_angles[0]
                theta = unwrapped_angles[1]

                # Proportional command
                command_angular_speed = self.kp_angle_init* (argument_target - theta)
                self.publish_command(linear = 0.0, angular = command_angular_speed)

                # Updating 
                unwrapped_angles = np.unwrap([argument_target, self.theta])
                argument_target = unwrapped_angles[0]
                theta = unwrapped_angles[1]

                # Printing infos
                print(f"Reaching start angle")
                print(f"Reference angle: {argument_target}")
                print(f"Current angle: {theta}")
                print(f"Error: {argument_target-theta}")
                print(f"Command: {self.command_angular_speed}\n\n")

                # Sleep to ensure sampling time
                self.loop_chrono.sleep()

            self.get_logger().info(f"Start angle reached !")

            # Stop robot
            self.publish_command(linear = 0.0, angular = 0.0)



        #-------------- Then reach the start point --------------

        if self.reachStartPoint: # Refaire la commande avec des arguments nommés
            # Launch controller as setpoint tracker to track the start point of the trajectory.
            subprocess.run(["python3", os.path.basename(__file__), "--tracking_type", str(self.SETPOINT_TRACKING), "--x", str(self.xref[0]), "--y", str(self.yref[0]), "--theta", str((math.atan2(self.yref[1]-self.yref[0],self.xref[1]-self.xref[0])%(2*math.pi))), "--topic_prefix", str(self.topic_prefix)])


        #-------------- And finally follow the trajectory/Reach the setpoint --------------

        # Starts setpoint or reference tracking
        while self.iref < self.imax or module_target > self.distance_error:
            # Updating time
            self.loop_chrono.start()
            start_time = time.time()

            # Verifying obstacle presence
            if self.SetObstacleAvoidance:
                if self.obstacle_distance != -1:
                    obstacle_angle = self.obstacle_angle
                    obstacle_distance = self.obstacle_distance
                    safety_margin = obstacle_distance*math.sin(math.radians(obstacle_angle))

                    print(f"margin:{safety_margin}, radius:{self.safety_radius}")
                    if abs(safety_margin)<self.safety_radius:
                        # If the robot continues forward it will collide the obstacle
                        print(f"Obstacle detected: EMERGENCY STOP")
                        self.publish_command(linear=0.0, angular=0.0)
                        obstacles = self.obstacles
                        obstacle = self.find_obstacle(obstacles, obstacle_angle)
                        print(f"Obstacle found: {obstacle}")
                        self.emergency_controller(obstacle)
                        #break # Break the controller and stop the robot
            
            # Computing tracking error
            eps_x = self.xref[self.iref] - self.x
            eps_y = self.yref[self.iref] - self.y

            # Computing module and argument to get to the target
            module_target = math.sqrt(eps_x ** 2 + eps_y ** 2)
            argument_target = math.atan2(eps_y, eps_x)

            # Wrapping argument to 2pi
            argument_target = argument_target%(2*math.pi)

            # Unwrapping current angle to argument target, to get the optimal angular command
            unwrapped_angles = np.unwrap([argument_target, self.theta])
            argument_target = unwrapped_angles[0]
            theta = unwrapped_angles[1]

           # Computing Integral term of PI controller
            self.integral_distance = self.integral_distance + self.ki_distance * (start_time - time_prev) * module_target
            self.integral_angle = self.integral_angle + self.ki_angle * (start_time - time_prev) * (argument_target - self.theta)

            # Command signal
            command_linear_speed = self.kp_distance * module_target + self.integral_distance
            command_angular_speed = self.kp_angle * (argument_target - theta) + self.integral_angle

            # Command publication
            self.publish_command(linear = command_linear_speed, angular = command_angular_speed)

            #Display distance to setpoint and angle error
            self.get_logger().info(f"Distance : {round(module_target, self.significant_numbers)}")
            self.get_logger().info(f"Angle error : {round(argument_target-self.theta, self.significant_numbers)}")
            self.get_logger().info(f"Delta t : {round(start_time - time_prev, self.significant_numbers)}")
            self.get_logger().info(f"step : {self.iref}/{self.imax}\n",)

            # Updating counter
            self.iref = min(self.iref + 1, self.imax) # When the terminal point has arrived, we keep it in time

            # Updating trajectory to plot and datas
            self.xplot.append(self.x)
            self.yplot.append(self.y)
            self.error.append(module_target)
            self.time_.append(start_time)
            segment = np.sqrt((self.x-self.x_prev) ** 2 +(self.y-self.y_prev) ** 2 )
            self.distance = self.distance + segment
            self.speeds.append(segment/(start_time-time_prev))

            # Memory stamp
            argument_prev = argument_target
            time_prev = start_time
            self.x_prev = self.x
            self.y_prev = self.y

            # Sleep to ensure sampling time
            self.loop_chrono.sleep()

        self.end_time = time.time()
        self.get_logger().info(f"Setpoint reached !")

        # Stop robot
        self.publish_command(linear = 0.0, angular = 0.0)

        # If we are in setpoint tracking mode, reach the final given angle.
        if self.tracking_type == self.SETPOINT_TRACKING:
            # Preparing for final angle reaching
            print(f"{self.theta_ref}   {self.theta}")
            unwrapped_angles = np.unwrap([self.theta_ref, self.theta])
            self.theta_ref = unwrapped_angles[0]
            theta = unwrapped_angles[1]

            while abs(self.theta_ref - theta) > self.angle_error:

                # Updating
                unwrapped_angles = np.unwrap([self.theta_ref, self.theta])
                self.theta_ref = unwrapped_angles[0]
                theta = unwrapped_angles[1]

                # Computing command
                command_angular_speed = 1*(self.theta_ref - theta)

                # Publish command
                self.publish_command(linear = 0.0, angular = command_angular_speed)

                # Print stats
                print(f"-- Reaching final angle --")
                print(f"Reference angle: {self.theta_ref}")
                print(f"Current angle: {theta}")
                print(f"Error: {self.theta_ref - theta}")
                print(f"Command: {command_angular_speed}\n\n")

            self.get_logger().info(f"Reference angle reached !")
        
        self.task_chrono.stop()

        self.get_logger().info(f"\nFinal datas :")
        self.get_logger().info(f"Average tracking error : {round(np.mean(self.error), self.significant_numbers)} [m]")
        self.get_logger().info(f"Task achieved in : {round(self.task_chrono.get_elapsed_time(), self.significant_numbers)} [s]")
        self.get_logger().info(f"Distance traveled : {round(self._distance, self.significant_numbers)} [m]")
        self.get_logger().info(f"Aveage speed : {round(np.mean(self._speeds), self.significant_numbers)} [m/s]")

        # Stop robot
        self.publish_command(linear = 0.0, angular = 0.0)

        # Adjusting vectors to plot
        while len(self.xref) < len(self._xplot):
            self.xref = np.concatenate([self.xref, [self.xref[-1]]])
        while len(self.yref) < len(self._yplot):
            self.yref = np.concatenate([self.yref, [self.yref[-1]]])

        self.tracking_finished = True
        self.get_logger().info(f"Controller succesfully ended")



    # Emergency controller

    def emergency_controller(self, obstacle: 'Obstacle'):
        # Stop robot
        self.publish_command(linear = 0.0, angular = 0.0)

        # Saving theta
        theta_start_avoiding = self.theta


        while obstacle is None:
            obstacle = self.find_obstacle(self.obstacles, self.obstacle_angle)
            time.sleep(0.2)

        safety_margins = obstacle.get_safety_margins()
        print(f"{safety_margins}")

        if np.mean(safety_margins)>0.0:
            ref = + self.safety_radius
            self.obstacle_avoidance = self.LEFT_AVOIDANCE
        else:
            ref = - self.safety_radius
            self.obstacle_avoidance = self.RIGHT_AVOIDANCE

        # Deviate only the orientation
        while any(abs(value) < self.safety_radius for value in safety_margins):
            start_time = time.time()

            # Computing command
            command_angular_speed = -5*(ref - min(safety_margins, key=abs))
            print(f"Min: {min(safety_margins, key=abs)}, Cmd: {ref-min(safety_margins)}")

            # Saturation
            command_angular_speed = min(max(command_angular_speed, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)

            # Command publication
            self.publish_command(linear = 0.0, angular = command_angular_speed)

            # Computation of the safety margin
            # safety_margin = self.obstacle_distance*math.sin(math.radians(self.obstacle_angle))
            
            obstacles = self.obstacles
            obstacle_angle = self.obstacle_angle
            obstacle = self.find_obstacle(obstacles, obstacle_angle)

            # It happens that the obstacle is not found due to the low precision of the lidar
            while obstacle is None:
                obstacles = self.obstacles
                obstacle_angle = self.obstacle_angle
                obstacle = self.find_obstacle(obstacles, obstacle_angle)
                time.sleep(0.2)

            
            safety_margins = obstacle.get_safety_margins()
            print(f"SM: {safety_margins}")

            end_time = time.time()
            if end_time - start_time<self.Ts:
                time.sleep(self.Ts-(end_time - start_time))
            else:
                time.sleep(self.Ts/10)

        # Stop robot
        self.publish_command(linear = 0.0, angular = 0.0)

        obstacles = self.obstacles
        obstacle_angle = self.obstacle_angle
        obstacle = self.find_obstacle(obstacles, obstacle_angle)

        # First verify in which obstacle belongs self.obstacle_angle and then verify that this obstacle is fully between 90° and 270°
        # while not (90 <= obstacle.start_angle <= 270) or not (90 <= obstacle.end_angle <= 270):
        #     start_time = time.time()

        #     # Move forward to pass the obstacle
        #     self.publish_command(linear = 0.15, angular = 0.0)

        #     print(f"Depassement d'obstacle")

        #     # Refreshing obstacle
        #     obstacles = self.obstacles
        #     obstacle_angle = self.obstacle_angle
        #     obstacle = self.find_obstacle(obstacles, obstacle_angle)

        #     end_time = time.time()
        #     if end_time - start_time < self.Ts:
        #         time.sleep(self.Ts - (end_time - start_time))




        ##  OTHER APPROACH ##
                
        # Preparing for angle reaching
        argument_target = math.atan2(self.yref[0] - self.y, self.xref[0] - self.x)
        unwrapped_angles = np.unwrap([argument_target, self.theta])
        argument_target = unwrapped_angles[0]
        theta = unwrapped_angles[1]

        obstacles = self.obstacles
        obstacle_angle = self.obstacle_angle
        obstacle = self.find_obstacle(obstacles, obstacle_angle)

        # It happens that the obstacle is not found due to the low precision of the lidar
        while obstacle is None:
            obstacles = self.obstacles
            obstacle_angle = self.obstacle_angle
            obstacle = self.find_obstacle(obstacles, obstacle_angle)
            time.sleep(0.2)

        while abs(argument_target - theta) > 50*self.angle_error:
            start_time = time.time()

            # Updating
            argument_target = math.atan2(self.yref[0] - self.y, self.xref[0] - self.x)
            unwrapped_angles = np.unwrap([argument_target, self.theta])
            argument_target = unwrapped_angles[0]
            theta = unwrapped_angles[1]

            # Computing command
            command_angular_speed = 1.2 * self.obstacle_avoidance*(self.safety_radius - self.obstacle_distance) # Keeping the TB close to the obstacle
            command_linear_speed = 0.15

            # Publish command
            self.publish_command(linear = command_linear_speed, angular = command_angular_speed)

            obstacles = self.obstacles
            obstacle_angle = self.obstacle_angle
            obstacle = self.find_obstacle(obstacles, obstacle_angle)

            # It happens that the obstacle is not found due to the low precision of the lidar
            while obstacle is None:
                obstacles = self.obstacles
                obstacle_angle = self.obstacle_angle
                obstacle = self.find_obstacle(obstacles, obstacle_angle)
                time.sleep(0.2)

            
            safety_margins = obstacle.get_safety_margins()

            end_time = time.time()
            if end_time - start_time < self.Ts:
                time.sleep(self.Ts - (end_time - start_time))



        ##  OTHER APPROACH ##

        # Preparing for angle reaching
        # unwrapped_angles = np.unwrap([theta_start_avoiding, self.theta])
        # argument_target = unwrapped_angles[0]
        # theta = unwrapped_angles[1]

        # obstacles = self.obstacles
        # obstacle_angle = self.obstacle_angle
        # obstacle = self.find_obstacle(obstacles, obstacle_angle)

        # # It happens that the obstacle is not found due to the low precision of the lidar
        # while obstacle is None:
        #     obstacles = self.obstacles
        #     obstacle_angle = self.obstacle_angle
        #     obstacle = self.find_obstacle(obstacles, obstacle_angle)
        #     time.sleep(0.2)

        # while abs(argument_target - theta) > 10 * self.angle_error :
        #     start_time = time.time()

        #     # Updating
        #     unwrapped_angles = np.unwrap([theta_start_avoiding, self.theta])
        #     argument_target = unwrapped_angles[0]
        #     theta = unwrapped_angles[1]

        #     # Computing command
        #     command_angular_speed = 0.7 * self.obstacle_avoidance*(self.safety_radius - self.obstacle_distance) # Keeping the TB close to the obstacle
        #     command_linear_speed = 0.1

        #     # Publish command
        #     self.publish_command(linear = command_linear_speed, angular = command_angular_speed)

        #     obstacles = self.obstacles
        #     obstacle_angle = self.obstacle_angle
        #     obstacle = self.find_obstacle(obstacles, obstacle_angle)

        #     # It happens that the obstacle is not found due to the low precision of the lidar
        #     while obstacle is None:
        #         obstacles = self.obstacles
        #         obstacle_angle = self.obstacle_angle
        #         obstacle = self.find_obstacle(obstacles, obstacle_angle)
        #         time.sleep(0.2)

            
        #     safety_margins = obstacle.get_safety_margins()

        #     end_time = time.time()
        #     if end_time - start_time < self.Ts:
        #         time.sleep(self.Ts - (end_time - start_time))


        print(f"Obstacle dépassé")

        # Stop robot
        self.publish_command(linear = 0.0, angular = 0.0)
        time.sleep(1) # Sleeping before assuming back the trajectory

        # Resetting obstacle avoidance to None
        self.obstacle_avoidance = None



        

    
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


    def rigid_bodies_callback(self, msg):
            # print(f"{msg}")
            x=msg.rigidbodies[6].pose.position.x
            y=msg.rigidbodies[6].pose.position.y
            z=msg.rigidbodies[6].pose.position.z

            qx = msg.rigidbodies[6].pose.orientation.x
            qy = msg.rigidbodies[6].pose.orientation.y
            qz = msg.rigidbodies[6].pose.orientation.z
            qw = msg.rigidbodies[6].pose.orientation.w

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


        #Then, we make the list of all obstacles
        # List of obstacles
        obstacles = []
        obstacle_start_angle = None
        for i in range(len(whole_region)):
            current_distance = whole_region[i]

            # Mapping the obstacle
            if current_distance <= self.obstacle_max_distance:
                # # Get Turtlebot distance to odom origin                
                # dist_to_origin = math.sqrt((self.x**2)+(self.y**2))

                # Get obstacle coordinates in robot coordinate system
                current_angle = (round((i)*360/nb_samples-self._180))%(360) 
                x_obs = (current_distance*math.cos(math.radians(current_angle)))
                y_obs = (current_distance*math.sin(math.radians(current_angle)))

                # Define the rotation matrix
                rotation_matrix = np.array([[np.cos(self.theta), -np.sin(self.theta)],[np.sin(self.theta), np.cos(self.theta)]])

                # Define the vector [x, y] for obstacle position in odom coordinate system
                obs_pose_robot = np.array([x_obs, y_obs])

                # Perform the matrix multiplication
                obs_pose_odom = np.dot(rotation_matrix, obs_pose_robot)

                self._xcarthography.append(obs_pose_odom[0] + self.x)    
                self._ycarthography.append(obs_pose_odom[1] + self.y)                
                          

            # Verify if the begining of an obstacle was found
            if current_distance <= self.obstacle_max_distance and obstacle_start_angle is None:
                obstacle_start_index = (i)%(nb_samples) # Getting a margin of a sampling_step
                obstacle_start_angle = (round((obstacle_start_index)*360/nb_samples-self._180))%(360) 

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

                obstacle = Obstacle(start_angle=obstacle_start_angle, end_angle=obstacle_end_angle, start_distance=whole_region[obstacle_start_index], end_distance=whole_region[obstacle_end_index])
                obstacles.append(obstacle)
                obstacle_start_angle = None # End of the obstacle, resetting the obstacle detection

        # After the loop, if an obstacle is still detected at the last angle:
        if (whole_region[-1]<=self.obstacle_max_distance) and (obstacle_start_angle is not None):
            obstacle_end_index = nb_samples - 1
            obstacle_end_angle = (round((obstacle_end_index)*360/nb_samples-self._180))%360

            # New obstacle ending at the last angle (close to 359°)
            obstacle = Obstacle(start_angle=obstacle_start_angle, end_angle=obstacle_end_angle, start_distance=whole_region[obstacle_start_index], end_distance=whole_region[obstacle_end_index])
            
            # And if the first angle also contains an obstacle
            if (whole_region[0] <= self.obstacle_max_distance):
                # Merging first and last obstacles into one
                obstacles[0].merge(obstacle)

            # And if the first angle doesn't contain anything, the obstacle finishes at the last angle.
            else:
                # Just adding a new obstacle
                obstacles.append(obstacle)

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
    def importFromCSV(self, filename: str):
        # Read CSV file with data
        df = pd.read_csv(filename, delimiter=';')

        # Extract x and y vectors
        x = df.iloc[:, 0].values  # First column
        y = df.iloc[:, 1].values  # Second column

        # Return x and y vectors
        return x, y


    def find_obstacle(self, obstacles, angle):
        # Retourne l'indice de l'obstacle auquel appartient l'angle donné
        for obstacle in obstacles:
            if obstacle.is_in_obstacle(angle):
                return obstacle
        return None  # Aucun obstacle trouvé pour l'angle donné
    



    # Function defined in teleop
    # Used to set constraints on command variations such that -slop<=Delta_u<=slop
    # output: Previous command
    # input: Computed command that may not respect constraints on variations
    # slop: Variation constraint

    def make_simple_profile(self, output, input, slop):
        # If the input is 0.0 we still set the command instantaneously
        # because we should be able to stop the robot immediately
        if input != 0.0: 
            if abs(input - output) > 0:     # Acceleration
                if input > output:
                    output = min(input, output + slop)
                else:
                    output = max(input, output - slop)
            # elif input < output and input < 0:
            #     output = max(input, output - slop) #Robot decelerates faster than it accelerates
            else:
                output = input
        else:
            output = 0.0
        return float(output)

    def publish_command(self, linear, angular):

        
        # Saturation
        command_linear_speed = min(max(linear, self.MIN_LINEAR_SPEED), self.MAX_LINEAR_SPEED)
        command_angular_speed = min(max(angular, self.MIN_ROTATION_SPEED), self.MAX_ROTATION_SPEED)

        # Input variation constraint
        self.command_linear_speed = self.make_simple_profile(self.command_linear_speed, command_linear_speed, self.LIN_VEL_STEP_SIZE)
        self.command_angular_speed = self.make_simple_profile(self.command_angular_speed, command_angular_speed, self.ANG_VEL_STEP_SIZE)

        # Publishing command
        self.cmd_msg.linear.x = self.command_linear_speed
        self.cmd_msg.linear.y = 0.0
        self.cmd_msg.linear.z = 0.0
        
        self.cmd_msg.angular.x = 0.0
        self.cmd_msg.angular.y = 0.0
        self.cmd_msg.angular.z = self.command_angular_speed

        self.publisher.publish(self.cmd_msg)

    # Used to sample & store position/speed/distance data for final stats & plots
    def data_saver(self):
        chrono = Chronometer(self.Ts)
        # Waiting for data publication
        while self.x is None or self.y is None:
            pass

        xprev = self.x
        yprev = self.y
        previous_time = time.time()
        previous_speed = 0

        self._speeds.append(0)
        self._accelerations.append(0)
        
        while not self.tracking_finished:
            chrono.start()
            current_time = time.time()

            self._time.append(current_time)

            # Updating position vector
            x = self.x
            y = self.y

            # Time step (normally = Ts)
            h = current_time-previous_time

            # Computing datas
            self._xplot.append(x)
            self._yplot.append(y)
            segment = math.sqrt((x-xprev)**2 + (y-yprev)**2)
            self._distance = self._distance + segment

            
            speed = segment/(h)
            self._speeds.append(speed)

            acceleration = (speed - previous_speed)/h
            self._accelerations.append(acceleration)


            # Data stamp
            xprev = x
            yprev = y
            previous_time = current_time
            previous_speed = speed


            chrono.sleep()



        
    


#####################################################################################
#                                   Main loop                                       #
#####################################################################################



def main(args=None):
    rclpy.init(args=args)
    node = MoveController()

    stop_publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    cmd_msg = Twist()

    cmd_msg.angular.z = 0.0
    cmd_msg.linear.x = 0.0
    

    try:
        # Start controller thread
        controller_thread = threading.Thread(target=node.controller)
        controller_thread.daemon = True  #  Allows the thread to end wen the node is finished
        controller_thread.start()

        # Start data saver controller
        data_saver_thread = threading.Thread(target = node.data_saver)
        data_saver_thread.daemon = True
        data_saver_thread.start()


        node.tracking_finished = False
        while rclpy.ok() and not node.tracking_finished:
            # Spin the node once to process any callbacks
            rclpy.spin_once(node, timeout_sec = 0.1)

    except Exception:
        stop_publisher.publish(cmd_msg)

    except KeyboardInterrupt:
        # Plot result even when interrupting function
        if node.setPlots:
            plt.plot(node._xplot, node._yplot, '.r')
            plt.plot(node._xcarthography, node._ycarthography, ',b')
            plt.legend(['Turtlebot Trajectory', 'Mapped obstacles'])
            plt.axis('scaled')
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.show()

            plt.plot(node.xplot, node.yplot, 'xr')
            plt.plot(node.xref, node.yref, '-b')
            plt.legend(['Turtlebot Trajectory', 'Reference'])
            plt.axis('scaled')
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.show()

            plt.plot(node.time_, node._xplot, 'xr')
            plt.plot(node.time_[:len(node.xref)], node.xref, '-b')
            plt.plot(node.time_, node._yplot, 'xr')
            plt.plot(node.time_[:len(node.yref)], node.yref, '-b')
            plt.legend(['X(t)','RefX(t)','Y(t)','RefY(t)'])
            plt.xlabel("Position [m]")
            plt.ylabel("Time [s]")
            plt.show()

            plt.plot(node._xplot, node._yplot, '.r')
            plt.plot(node.xref, node.yref, '-b')
            plt.legend(['Turtlebot Trajectory', 'Reference'])
            plt.axis('scaled')
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.show()

        stop_publisher.publish(cmd_msg)
        pass
    
    # Wait for the thread to stop
    node.get_logger().info(f"joining thread")
    data_saver_thread.join()
    controller_thread.join()
    

    # Plot result
    
    if node.setPlots:
        plt.plot(node._xplot, node._yplot, '.r')
        plt.plot(node._xcarthography, node._ycarthography, ',b')
        plt.legend(['Turtlebot Trajectory', 'Mapped obstacles'])
        plt.axis('scaled')
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.show()

        plt.plot(node._xplot, node._yplot, '.r')
        plt.plot(node.xref, node.yref, '-b')
        plt.legend(['Turtlebot Trajectory', 'Reference'])
        plt.axis('scaled')
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.show()

        plt.plot(node._xplot, node._yplot, 'xr')
        plt.plot(node.xref, node.yref, '-b')
        plt.axis('scaled')
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.show()

        plt.plot(node.time_, node.xplot, 'xr')
        plt.plot(node.time_[:len(node.xref)], node.xref, '-b')
        plt.plot(node.time_, node.yplot, 'xr')
        plt.plot(node.time_[:len(node.yref)], node.yref, '-b')
        plt.xlabel("Position [m]")
        plt.ylabel("Time [s]")
        plt.show()
    
    stop_publisher.publish(cmd_msg)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
