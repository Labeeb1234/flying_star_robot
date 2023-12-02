#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from tf_transformations import euler_from_quaternion
import numpy as np
from transformation.srv import Transform  # Import custom service message


class TransformationController(Node):

    def __init__(self):
        super().__init__("transformation_controller")

        self.callback_group = ReentrantCallbackGroup()

        self.left_arm_pub = self.create_publisher(Wrench, "fs_bot/left_arm_force", 10)
        self.right_arm_pub = self.create_publisher(Wrench, "fs_bot/right_arm_force", 10)

        self.left_arm_pose_sub = self.create_subscription(Odometry, "fs_bot/left_arm_pose", self.left_arm_angle_callback, 10, callback_group=self.callback_group)
        self.right_arm_pose_sub = self.create_subscription(Odometry, "fs_bot/right_arm_pose", self.right_arm_angle_callback, 10, callback_group=self.callback_group)

        self.transformation_service = self.create_service(Transform, "fs_bot/transform", self.transformation_callback ,callback_group=self.callback_group)
        
        self.left_arm_angle = 0.0
        self.right_arm_angle = 0.0
        self.desired_arm_angle = 0.0
        self.is_transform = True
        self.need_tranform = True
        self.is_transformed = False
        self.torque_val = 0.0
        
        self.arm_transformer = self.create_timer(0.02, self.arm_controller)
        self.arm_transformer.cancel()

    def left_arm_angle_callback(self, left_arm_msg):
        quat_x = left_arm_msg.pose.pose.orientation.x
        quat_y = left_arm_msg.pose.pose.orientation.y
        quat_z = left_arm_msg.pose.pose.orientation.z
        quat_w = left_arm_msg.pose.pose.orientation.w
        
        roll, pitch, yaw = euler_from_quaternion([quat_x, quat_y, quat_z ,quat_w])
        
        self.left_arm_angle = roll
        # self.get_logger().info(f"left_roll_angle: {self.left_arm_angle}")
    
    def right_arm_angle_callback(self, right_arm_msg):
        quat_x = right_arm_msg.pose.pose.orientation.x
        quat_y = right_arm_msg.pose.pose.orientation.y
        quat_z = right_arm_msg.pose.pose.orientation.z
        quat_w = right_arm_msg.pose.pose.orientation.w
        
        roll, pitch, yaw = euler_from_quaternion([quat_x, quat_y, quat_z ,quat_w])

        self.right_arm_angle = roll
        # self.get_logger().info(f"left_roll_angle: {self.left_arm_angle}")

    def arm_controller(self):
        left_torque_msg = Wrench()
        right_torque_msg = Wrench()

        if(self.is_transform == True):

            left_torque_msg.force.x = 0.0
            left_torque_msg.force.y = 0.0
            left_torque_msg.force.z = 0.0

            left_torque_msg.torque.x = self.torque_val
            left_torque_msg.torque.y = 0.0
            left_torque_msg.torque.z = 0.0

            right_torque_msg.force.x = 0.0
            right_torque_msg.force.y = 0.0
            right_torque_msg.force.z = 0.0

            right_torque_msg.torque.x = -self.torque_val
            right_torque_msg.torque.y = 0.0
            right_torque_msg.torque.z = 0.0



            self.left_arm_pub.publish(left_torque_msg)
            self.right_arm_pub.publish(right_torque_msg)


            if(np.abs(self.left_arm_angle) >= self.desired_arm_angle and np.abs(self.right_arm_angle) >= self.desired_arm_angle):
                self.is_transformed = True
                self.is_transform = False
                self.arm_transformer.cancel()
            else:
                self.is_transformed = False
                self.is_transform = True


    def transformation_callback(self, request, response):
        self.need_transform = request.transform
        self.desired_arm_angle = request.angle
        self.torque_val = request.torque

        if(self.need_transform == True):
            self.is_transformed = False
            self.is_transform = True
            self.arm_transformer.reset()
        else:
            self.arm_transformer.cancel()
            response.success = False
            response.message = "Transformation Incomplete"

        self.get_logger().info("transformation_started")
        rate = self.create_rate(2, self.get_clock())

        while not self.is_transformed:
            self.get_logger().info("Waiting for transformation...")
            rate.sleep()

        self.get_logger().info("service action over...!")

        response.success = True
        response.message = "Transformation done"
        return response


def main(args = None):
    rclpy.init(args=args)

    transformation_controller_node = TransformationController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)  
    executor.add_node(transformation_controller_node)

    executor.spin()


    rclpy.shutdown()
    exit(0)

if __name__ == 'main':
    main()


