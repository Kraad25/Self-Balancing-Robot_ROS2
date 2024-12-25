#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from control import lqr
from std_msgs.msg import Float32, String


# System and control matrices
A = np.array([[0, 1], [5, 0]])  # Adjusted to increase the effect of tilt angle
B = np.array([[0], [1.5]])         # Increase B to make the control more responsive

class LQR(Node):
    def __init__(self):    
        super().__init__("lqr_values")
        self.get_logger().info("LQR is Computing Values")

        self.lqr_inp = self.create_subscription(Imu, '/lqr_input', self.lqrCallback, 10)
        self.lqr_output = self.create_publisher(Twist, '/motor_cmds', 10)
        self.yaw_pub = self.create_publisher(Float32, "/yaw_monitor", 10)  # New publisher for yaw

        
        self.Q = np.array([[5, 0], [0, 15]])
        self.R = 0.1
        self.K, self.S, self.e = lqr(A, B, self.Q, self.R)

    def lqrCallback(self, data: Imu):
        # Recalculate LQR controller with potential updated Q and R if needed
        self.K, self.S, self.e = lqr(A, B, self.Q, self.R)
    
        # Read tilt and angular velocity directly from IMU message
        tilt_angle = data.orientation.y * 180 / 3.1416  # Convert pitch (tilt) to degrees
        tilt_rate = data.angular_velocity.y  # Angular velocity around Y-axis

        np_x = np.array([[tilt_angle], [tilt_rate]])
        # Calculate feedback gain for tilt
        feedback = - self.K.dot(np_x)[0, 0]

        # Set a threshold for the tilt angle to stop applying corrective velocities
        if abs(tilt_angle) < 1.0:  # Within 1 degree tolerance
            feedback = 0.0  # Stop corrective action if balanced
        motor_vel = Twist()
        motor_vel.linear.x = float(feedback)
        motor_vel.angular.z = 0.0

        self.lqr_output.publish(motor_vel)

        yaw_msg = Float32()
        yaw_msg.data = tilt_angle
        self.yaw_pub.publish(yaw_msg)


if __name__ == '__main__':
    rclpy.init()
    lqr_values = LQR()
    rclpy.spin(lqr_values)
    lqr_values.destroy_node()
    rclpy.shutdown()
