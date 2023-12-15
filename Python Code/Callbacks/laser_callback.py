#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
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

from rclpy.qos import qos_profile_sensor_data

class laser_Callback(Node):
    #####################################################################################
    #                                Initialization                                     #
    #####################################################################################
    def __init__(self):
        super().__init__('laser_callback')

        self.sub = self.create_subscription(LaserScan, sys.argv[1] +'/scan', self.laser_callback, qos_profile = qos_profile_sensor_data)
         
    #####################################################################################
    #                                   Callback                                        #
    #####################################################################################
    
    def laser_callback(self, msg):
        obstacle_angle, obstacle_distance = min(enumerate(msg.ranges), key=lambda x: x[1])
        nb = len(msg.ranges)
        os.system('clear')
        print(f"Distance: {round(obstacle_distance, 4)}, angle = {round((obstacle_angle*360/nb-180)%360, 4)}, nb of angles = {nb}")

            

#####################################################################################
#                                   Main loop                                       #
#####################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = laser_Callback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()