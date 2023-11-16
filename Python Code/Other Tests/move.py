#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
class  move(Node):
    def  __init__(self):
        super().__init__('test_node')
        self.publisher=self.create_publisher(Twist,'cmd_vel',10)
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.cmd_msg=Twist()
        self.start_time=time.time()
    
    def timer_callback(self):
        current_time=time.time()
        if current_time - self.start_time < 10.0:
           self.cmd_msg.linear.x=0.2
           self.publisher.publish(self.cmd_msg)
        else:
            self.cmd_msg.linear.x=0.0
            self.publisher.publish(self.cmd_msg)
    
def main(args=None):
        rclpy.init(args=args)
        node=move()
        try : 
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__' :
    main() 