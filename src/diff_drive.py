#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class DifferentialDrive(Node):
    def __init__(self):
        super().__init__('differential_drive')
        self.get_logger().info("Differential Drive Node Started")

        self.motor_sub = self.create_subscription(Twist, '/motor_cmds', self.motor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot parameters
        self.wheel_base = 1.17172  # Distance between wheels
        self.wheel_radius = 0.35  # Radius of wheels

    def motor_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate left and right wheel speeds
        left_wheel, right_wheel = self.compute_wheel_speeds(linear_x, angular_z)

        # Publish to cmd_vel to move robot
        cmd_msg = Twist()
        cmd_msg.linear.x = (left_wheel + right_wheel) * self.wheel_radius / 2
        cmd_msg.angular.z = (right_wheel - left_wheel) * self.wheel_radius / self.wheel_base
        
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"Left: {left_wheel:.2f}, Right: {right_wheel:.2f}")

    def compute_wheel_speeds(self, linear_x, angular_z):
        left_wheel = (linear_x - angular_z * self.wheel_base / 2) / self.wheel_radius
        right_wheel = (linear_x + angular_z * self.wheel_base / 2) / self.wheel_radius
        return left_wheel, right_wheel


if __name__ == '__main__':
    rclpy.init()
    node = DifferentialDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()