#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node") # type: ignore
        self.counter_ = 0
        self.timer_1 = self.create_timer(1.0, self.timer_callback)
        self.timer_2 = self.create_timer(1.0, self.timer_2_callback)
    def timer_callback(self):
        self.get_logger().info("Hello" + str(self.counter_))
        self.counter_ += 1

    def timer_2_callback(self):
        self.get_logger().info("I am robot")

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 