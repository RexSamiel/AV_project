#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode():

    def __init__(self):
        super().__init__("first_node")
        self._counter = 0
        self.get_logger().info("Helllloooo")

    def timer_callback(self):
        self.get_logger().info("Hewwoo!" + str(self._counter))
        self._counter += 1
        

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    
    rclpy.spin(node) 
    rclpy.shutdown()
    pass

if __name__ == '__main__':
    main()