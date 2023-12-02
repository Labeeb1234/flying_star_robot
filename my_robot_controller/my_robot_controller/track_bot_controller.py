#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
import numpy as np


class TrackBotController(Node):

    def __init__(self):
        super().__init__("track_bot_controller")

        self.twist_sub = self.create_subscription(Twist, "fs_bot/cmd_vel", self.motion_callback, 10)
        self.sheild_1_pub = self.create_publisher(Wrench, "fs_bot/shield_motion_1", 10)


        self.bot_u = 0.0
        self.bot_ang = 0.0

    def motion_callback(self, twist_msg):
        self.bot_u = twist_msg.linear.x
        self.bot_ang = twist_msg.angular.z

    def bot_inverse_kinematics(self, u, v, r):
        return



def main(args = None):
    rclpy.init(agrs=args)

    track_bot_controller_node = TrackBotController()

    rclpy.shutdown()
    exit(0)

if __name__ == 'main':
    main()


        