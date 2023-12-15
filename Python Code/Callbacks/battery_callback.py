#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
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

from rclpy.qos import qos_profile_sensor_data

class battery_Callback(Node):
    #####################################################################################
    #                                Initialization                                     #
    #####################################################################################
    def __init__(self):
        super().__init__('battery_callback')

        self.sub = self.create_subscription(BatteryState, sys.argv[1] + '/battery_state', self.battery_callback, qos_profile = qos_profile_sensor_data)
         
    #####################################################################################
    #                                   Callback                                        #
    #####################################################################################
    
    def battery_callback(self, msg):
        self.battery = msg.percentage

            

#####################################################################################
#                                   Main loop                                       #
#####################################################################################

def main(args=None):
    rclpy.init(args=args)
    node = battery_Callback()
    rclpy.spin_once(node)
    battery = node.battery
    print(f"{round(battery, 4)}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()