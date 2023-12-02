#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
import numpy as np

class LeftArmController(Node):

    def __init__(self):
        super().__init__("left_arm_controller") 
        
        self.callback_group = ReentrantCallbackGroup()
        self.left_arm_pub = self.create_publisher(Wrench, "fs_bot/left_arm_force", 10)
        self.left_arm_orientation = self.create_subscription(Odometry, "fs_bot/left_arm_pose", self.left_arm_angle_callback, 10, callback_group=self.callback_group)

        self.timer_1 = self.create_timer(0.02, self.transform_callback)

        self.get_logger().info("Running left arm controller node")

        # required variables for the class
        self.left_arm_angle = 0.0

    def left_arm_angle_callback(self, left_arm_msg):
        quat_x = left_arm_msg.pose.pose.orientation.x
        quat_y = left_arm_msg.pose.pose.orientation.y
        quat_z = left_arm_msg.pose.pose.orientation.z
        quat_w = left_arm_msg.pose.pose.orientation.w
        
        roll, pitch, yaw = euler_from_quaternion([quat_x, quat_y, quat_z ,quat_w])
        
        # self.get_logger().info(f"left_roll_angle: {roll}")
        self.left_arm_angle = roll

    def transform_callback(self):
        arm_msg = Wrench()

        arm_msg.force.x = 0.0
        arm_msg.force.y = 0.0
        arm_msg.force.z = 0.0

        arm_msg.torque.x = 2.0
        arm_msg.torque.y = 0.0
        arm_msg.torque.z = 0.0


        self.left_arm_pub.publish(msg=arm_msg)
        

class RightArmController(Node):

    def __init__(self):
        super().__init__("right_arm_controller")  
        
        self.callback_group = ReentrantCallbackGroup()
        self.right_arm_pub = self.create_publisher(Wrench, "fs_bot/right_arm_force", 10)
        self.right_arm_orientation = self.create_subscription(Odometry, "fs_bot/right_arm_pose", self.right_arm_angle_callback, 10, callback_group=self.callback_group)

        self.create_timer(0.02, self.transform_callback)

        self.get_logger().info("Running right arm controller node")

        # required variables for the class
        self.right_arm_angle = 0.0

    def right_arm_angle_callback(self, right_arm_msg):
        quat_x = right_arm_msg.pose.pose.orientation.x
        quat_y = right_arm_msg.pose.pose.orientation.y
        quat_z = right_arm_msg.pose.pose.orientation.z
        quat_w = right_arm_msg.pose.pose.orientation.w
        
        roll, pitch, yaw = euler_from_quaternion([quat_x, quat_y, quat_z ,quat_w])
        # self.get_logger().info(f"right_roll_angle: {roll}")
        self.right_arm_angle = roll
       

    def transform_callback(self):
        arm_msg = Wrench()
        
        arm_msg.force.x = 0.0
        arm_msg.force.y = 0.0
        arm_msg.force.z = 0.0

        arm_msg.torque.x = -2.0
        arm_msg.torque.y = 0.0
        arm_msg.torque.z = 0.0

        self.right_arm_pub.publish(msg=arm_msg)


def main(args = None):
    rclpy.init(args=args)

    left_arm_controller_node = LeftArmController()
    right_arm_controller_node = RightArmController()

    executor = rclpy.executors.MultiThreadedExecutor(2)  
    executor.add_node(left_arm_controller_node)
    executor.add_node(right_arm_controller_node)


    executor.spin()

    rclpy.shutdown()
    exit(0)

if __name__ == 'main':
    main()
