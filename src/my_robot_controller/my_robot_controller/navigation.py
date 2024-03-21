#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
import tf_transformations


class TurtlebotNavigationNode(Node):

    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our controller is started")

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        
        self._pose_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10)
        
        self._odom_listener = self.create_subscription(
            Odometry, "/odom", self.robot_controller, 10)
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0

        qq = tf_transformations.quaternion_from_ruler(0,0,90)
        initial_pose.pose.pose.position.x = qq[0]
        initial_pose.pose.pose.position.y = qq[1]
        initial_pose.pose.pose.position.z = qq[2]
        initial_pose.pose.pose.position.w = qq[3]
        self.initial_pose_publisher.publish(initial_pose)
        

    def robot_controller(self,scan : LaserScan):
        cmd = Twist()
        a = 2 # This number increases the number of readings in each direction
        self._front = min(scan.ranges[:a+1] + scan.ranges[-a:]) # Reading from the beginning and end of the list
        self._left = min(scan.ranges[90-a :90+a+1])
        self._back = min(scan.ranges[180-a :180+a+1])
        self._right = min(scan.ranges[270-a :270+a+1])
        if self._front < 1.0:
            if self._right < self._left:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.5
            else:
                cmd.linear.x = 0.05
                cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        self._pose_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()