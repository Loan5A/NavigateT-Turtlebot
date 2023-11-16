import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import numpy as np
from math import atan2, sqrt
from tf2_ros import Buffer, TransformListener

class CustomNavigationNode(Node):
    def __init__(self):
        super().__init__('custom_navigation_node')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tfBuffer = Buffer()
        self.odom_frame = 'odom'  # Mettez à jour avec l'ID de votre frame odom
        self.base_frame = 'base_link'  # Mettez à jour avec l'ID de votre frame base
        self.kp_distance = 0.9  # Ajustez ce gain au besoin
        self.kp_angle = 0.9  # Ajustez ce gain au besoin
        self.MAX_LINEAR_SPEED = 0.22  # Vitesse linéaire maximale
        self.MIN_LINEAR_SPEED = -0.22  # Vitesse linéaire minimale
        self.MAX_ROTATION_SPEED = 2.84  # Vitesse de rotation maximale
        self.MIN_ROTATION_SPEED = -2.84  # Vitesse de rotation minimale

    def run(self):
        move_cmd = Twist()
        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        goal_x, goal_y, goal_z = self.getkey()
        if goal_z > 180 or goal_z < -180:
            self.get_logger().info("Vous avez entré une plage incorrecte pour z.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        previous_position, previous_angle = self.get_odom()

        while distance > 0.05:
            #Get position and angle
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            #unwrapping angle
            tab_angles = [previous_angle, path_angle]
            angles_unwrap = np.unwrap(tab_angles)
            path_angle = angles_unwrap[1]

            #For derivative
            #diff_angle = angles_unwrap[1] - previous_angle
            #diff_distance = distance - previous_distance 

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = min(kp_distance*distance,MAX_LINEAR_SPEED)
            control_signal_distance = min(control_signal_distance,MIN_LINEAR_SPEED)

            control_signal_angle = min(kp_angle * (path_angle - rotation),MAX_ROTATION_SPEED)
            control_signal_angle = max(control_signal_angle,MIN_ROTATION_SPEED)

            
            move_cmd.angular.z = control_signal_angle
            move_cmd.linear.x = control_signal_distance

            #if move_cmd.angular.z > 0:
            #    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            #else:
            #    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            self.cmd_vel.publish(move_cmd)
            rclpy.spin_once(self.node)
            previous_angle = rotation


        (position, rotation) = self.get_odom()

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - np.pi:
                    move_cmd = Twist()
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd = Twist()
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + np.pi and rotation > goal_z:
                    move_cmd = Twist()
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd = Twist()
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            rclpy.spin_once(self.node)

        self.get_logger().info("Arrêt du robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x = float(input("Entrez la coordonnée X (m) : "))
        y = float(input("Entrez la coordonnée Y (m) : "))
        z = float(input("Entrez l'orientation Z (degré, -180 ~ 180) : "))
        return x, y, z

    def get_odom(self):
        transform = self.tfBuffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
        trans = transform.transform.translation
        rotation = transform.transform.rotation
        return (Point(trans.x, trans.y, trans.z), rotation)

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rclpy.spin_once(self)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CustomNavigationNode()
    node.run()
    rclpy.spin(node)
    node.shutdown()

if __name__ == '__main__':
    main()
