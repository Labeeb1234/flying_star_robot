#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
import numpy as np


class DroneStablizer(Node):

    def __init__(self):
        super().__init__("drone_stablizer")
        self.drone_stablizer_pub = self.create_publisher(Wrench, "fs_bot/drone_stabilizer", 10)
        self.twist_msg_sub = self.create_subscription(Twist, "cmd_vel", self.twist_msg_callback, 10)

        self.timer = self.create_timer(0.02, self.drone_stabilizer)
        
        self.drone_up_vel = 0.0
        self.drone_fwd_vel = 0.0
        self.drone_side_vel = 0.0
    
    def twist_msg_callback(self, twist_msg):
        self.drone_up_vel = twist_msg.linear.x
        self.drone_side_vel = twist_msg.linear.y

    def drone_stabilizer(self):
        drone_force_msg = Wrench()

        drone_force_msg.force.x = 0.0
        drone_force_msg.force.y = 0.0
        drone_force_msg.force.z = self.drone_up_vel

        drone_force_msg.torque.x = 0.0
        drone_force_msg.torque.y = 0.0
        drone_force_msg.torque.z = 0.0

        self.drone_stablizer_pub.publish(drone_force_msg)

def main(args = None):
    rclpy.init(args=args)

    drone_stablizer_node = DroneStablizer()

    rclpy.spin(drone_stablizer_node)

    rclpy.shutdown()
    exit(0)

if __name__ == 'main':
    main()