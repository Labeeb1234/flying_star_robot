#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion
import numpy as np
from sensor_msgs.msg import Imu
import sys

class Motor1(Node):

    def __init__(self, motor_speed, motor_torque):
        super().__init__("motor1")  
        self.callback_group = ReentrantCallbackGroup()
        
        self.motor_1_pub = self.create_publisher(Wrench, "fs_bot/drone_motor_1", 10)
        self.motor_1_twist_sub = self.create_subscription(Twist, "cmd_vel", self.twist_msg_callback, 10)
        
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_angle_callback, 10, callback_group=self.callback_group)

        self.timer_1 = self.create_timer(0.02, self.motor_force_callback)
        self.get_logger().info("starting motor_1 controller....")

        self.input_sig = motor_speed
        self.motor_rotor_sig = motor_torque

        self.kp = -50.0
        self.kp_1 = 50.0
        self.kp_2 = 10.0

        self.motor_1_vel_x = 0.0
        self.motor_1_vel_y = 0.0
        self.motor_ang_z = 0.0
        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0

    
    def imu_angle_callback(self, imu_msg):
        imu_x = imu_msg.orientation.x
        imu_y = imu_msg.orientation.y
        imu_z = imu_msg.orientation.z
        imu_w = imu_msg.orientation.w        

        imu_roll, imu_pitch, imu_yaw = euler_from_quaternion([imu_x, imu_y, imu_z, imu_w])
        self.drone_roll = imu_roll
        self.drone_pitch = imu_pitch
        self.drone_yaw = imu_yaw
        self.get_logger().info(f"roll_angle: {self.drone_roll}, pitch_angle: {self.drone_pitch}, yaw_angle: {self.drone_yaw}")


    def twist_msg_callback(self, twist_msg):
        self.motor_1_vel_x = twist_msg.linear.x 
        self.motor_1_vel_y = twist_msg.linear.y
        self.motor_ang_z = twist_msg.angular.z

    def motor_force_callback(self):
        motor_force = Wrench()

        motor_force.force.x = 0.0
        motor_force.force.y = 0.0
        motor_force.force.z = self.input_sig + self.kp*(self.motor_1_vel_x-self.drone_pitch)  + self.kp_1*(self.motor_1_vel_y-self.drone_roll) + self.kp_2*(self.motor_ang_z-self.drone_yaw)

        motor_force.torque.x = 0.0
        motor_force.torque.y = 0.0
        motor_force.torque.z = self.motor_rotor_sig

        self.motor_1_pub.publish(motor_force)

class Motor2(Node):

    def __init__(self, motor_speed, motor_torque):
        super().__init__("motor2")     
        self.callback_group = ReentrantCallbackGroup()

        self.motor_2_pub = self.create_publisher(Wrench, "fs_bot/drone_motor_2", 10)
        self.motor_2_twist_sub = self.create_subscription(Twist, "cmd_vel", self.twist_msg_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_angle_callback, 10, callback_group=self.callback_group)

        self.timer_2 = self.create_timer(0.02, self.motor_force_callback)
        self.get_logger().info("starting motor_2 controller....")

        self.input_sig = motor_speed
        self.motor_rotor_sig = motor_torque

        self.kp = 50.0
        self.kp_1 = 50.0
        self.kp_2 = 10.0

        self.motor_2_vel_x = 0.0
        self.motor_2_vel_y = 0.0
        self.motor_2_ang_z = 0.0
        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0
        
    
    def imu_angle_callback(self, imu_msg):
         

        imu_x = imu_msg.orientation.x
        imu_y = imu_msg.orientation.y
        imu_z = imu_msg.orientation.z
        imu_w = imu_msg.orientation.w        

        imu_roll, imu_pitch, imu_yaw = euler_from_quaternion([imu_x, imu_y, imu_z, imu_w])
        self.drone_roll = imu_roll
        self.drone_pitch = imu_pitch
        self.drone_yaw = imu_yaw
        self.get_logger().info(f"roll_angle: {self.drone_roll}, pitch_angle: {self.drone_pitch}, yaw_angle: {self.drone_yaw}")

    def twist_msg_callback(self, twist_msg):
        self.motor_2_vel_x = twist_msg.linear.x 
        self.motor_2_vel_y = twist_msg.linear.y
        self.motor_2_ang_z = twist_msg.angular.z

    def motor_force_callback(self):
        motor_force = Wrench()

        motor_force.force.x = 0.0
        motor_force.force.y = 0.0
        motor_force.force.z = self.input_sig + self.kp*(self.motor_2_vel_x-self.drone_pitch) + self.kp_1*(self.motor_2_vel_y-self.drone_roll) + self.kp_2*(self.motor_2_ang_z-self.drone_yaw)

        motor_force.torque.x = 0.0
        motor_force.torque.y = 0.0
        motor_force.torque.z = self.motor_rotor_sig

        self.motor_2_pub.publish(motor_force)

class Motor3(Node):

    def __init__(self, motor_speed, motor_torque):
        super().__init__("motor3")     
        self.callback_group = ReentrantCallbackGroup()

        self.motor_3_pub = self.create_publisher(Wrench, "fs_bot/drone_motor_3", 10)
        self.motor_3_twist_sub = self.create_subscription(Twist, "cmd_vel", self.twist_msg_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_angle_callback, 10, callback_group=self.callback_group)

        self.timer_3 = self.create_timer(0.02, self.motor_force_callback)
        self.get_logger().info("starting motor_3 controller....")

        self.input_sig = motor_speed
        self.motor_rotor_sig = motor_torque

        self.kp = -50.0
        self.kp_1 = -50.0
        self.kp_2 = 10.0

        self.motor_3_vel_x = 0.0
        self.motor_3_vel_y = 0.0
        self.motor_3_ang_z = 0.0
        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0
    
    def imu_angle_callback(self, imu_msg):
         

        imu_x = imu_msg.orientation.x
        imu_y = imu_msg.orientation.y
        imu_z = imu_msg.orientation.z
        imu_w = imu_msg.orientation.w        

        imu_roll, imu_pitch, imu_yaw = euler_from_quaternion([imu_x, imu_y, imu_z, imu_w])
        self.drone_roll = imu_roll
        self.drone_pitch = imu_pitch
        self.drone_yaw = imu_yaw
        self.get_logger().info(f"roll_angle: {self.drone_roll}, pitch_angle: {self.drone_pitch}, yaw_angle: {self.drone_yaw}")

    def twist_msg_callback(self, twist_msg):
        self.motor_3_vel_x = twist_msg.linear.x 
        self.motor_3_vel_y = twist_msg.linear.y
        self.motor_3_ang_z = twist_msg.angular.z

    def motor_force_callback(self):
        motor_force = Wrench()

        motor_force.force.x = 0.0
        motor_force.force.y = 0.0
        motor_force.force.z = self.input_sig + self.kp*(self.motor_3_vel_x-self.drone_pitch) + self.kp_1*(self.motor_3_vel_y-self.drone_roll) + self.kp_2*(self.motor_3_ang_z-self.drone_yaw)

        motor_force.torque.x = 0.0
        motor_force.torque.y = 0.0
        motor_force.torque.z = self.motor_rotor_sig

        self.motor_3_pub.publish(motor_force)


class Motor4(Node):

    def __init__(self, motor_speed, motor_torque):
        super().__init__("motor4")     
        self.callback_group = ReentrantCallbackGroup()

        self.motor_4_pub = self.create_publisher(Wrench, "fs_bot/drone_motor_4", 10)
        self.motor_4_twist_sub = self.create_subscription(Twist, "cmd_vel", self.twist_msg_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_angle_callback, 10, callback_group=self.callback_group)

        self.timer_4 = self.create_timer(0.02, self.motor_force_callback)
        self.get_logger().info("starting motor_4 controller....")

        self.input_sig = motor_speed
        self.motor_rotor_sig = motor_torque

        self.kp = 50.0
        self.kp_1 = -50.0
        self.kp_2 = 10.0

        self.motor_4_vel_x = 0.0
        self.motor_4_vel_y = 0.0
        self.motor_4_ang_z = 0.0
        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0
    
    def imu_angle_callback(self, imu_msg):
         

        imu_x = imu_msg.orientation.x
        imu_y = imu_msg.orientation.y
        imu_z = imu_msg.orientation.z
        imu_w = imu_msg.orientation.w        

        imu_roll, imu_pitch, imu_yaw = euler_from_quaternion([imu_x, imu_y, imu_z, imu_w])
        self.drone_roll = imu_roll
        self.drone_pitch = imu_pitch
        self.drone_yaw = imu_yaw
        self.get_logger().info(f"roll_angle: {self.drone_roll}, pitch_angle: {self.drone_pitch}, yaw_angle: {self.drone_yaw}")
    
    def twist_msg_callback(self, twist_msg):
        self.motor_4_vel_x = twist_msg.linear.x
        self.motor_4_vel_y = twist_msg.linear.y
        self.motor_4_ang_z = twist_msg.angular.z 

    def motor_force_callback(self):
        motor_force = Wrench()

        motor_force.force.x = 0.0
        motor_force.force.y = 0.0
        motor_force.force.z = self.input_sig + self.kp*(self.motor_4_vel_x-self.drone_pitch) + self.kp_1*(self.motor_4_vel_y-self.drone_roll) + self.kp_2*(self.motor_4_ang_z-self.drone_yaw)

        motor_force.torque.x = 0.0
        motor_force.torque.y = 0.0
        motor_force.torque.z = self.motor_rotor_sig

        self.motor_4_pub.publish(motor_force)

def main():
    rclpy.init()

    motor_force_val = float(sys.argv[1])
    motor_torque_val = float(sys.argv[2])

    motor_1_node = Motor1(motor_speed=motor_force_val, motor_torque=motor_torque_val)
    motor_2_node = Motor2(motor_speed=motor_force_val, motor_torque=motor_torque_val)
    motor_3_node = Motor3(motor_speed=motor_force_val, motor_torque=motor_torque_val)
    motor_4_node = Motor4(motor_speed=motor_force_val, motor_torque=motor_torque_val)


    executor = rclpy.executors.MultiThreadedExecutor(4)     
    executor.add_node(motor_1_node)
    executor.add_node(motor_2_node)
    executor.add_node(motor_3_node)
    executor.add_node(motor_4_node)

    executor.spin()

    rclpy.shutdown()
    exit(0)

if __name__ == 'main':
    main()

