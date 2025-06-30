#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np
from control import lqr

# System and control matrices
A = np.array([[0, 1], [5, 0]])
B = np.array([[0], [9.0]])
MAX_TILT_ANGLE = 45.0  # Stop correction beyond 45 degrees

class LQR(Node):
    def __init__(self):    
        super().__init__("lqr_values")
        self.get_logger().info("LQR is Computing Values")

        self.lqr_inp = self.create_subscription(Imu, '/lqr_input', self.lqrCallback, 10)
        self.lqr_output = self.create_publisher(Twist, '/motor_cmds', 10)

        self.Q = np.array([[20, 0], [0, 50]])
        self.R = 2.0
        self.K, self.S, self.e = lqr(A, B, self.Q, self.R)
        
    def lqrCallback(self, data: Imu): 
        # Read tilt and angular velocity directly from IMU message
        tilt_angle = data.orientation.y * 180 / 3.1416  # Convert pitch (tilt) to degrees
        tilt_rate = data.angular_velocity.y 

        np_x = np.array([[tilt_angle], [tilt_rate]])
        # Calculate feedback gain for tilt
        feedback = - self.K.dot(np_x)[0, 0]

        # Stop if tilt is beyond the threshold
        if abs(tilt_angle) > MAX_TILT_ANGLE:
            feedback = 0.0       
        if -3.0 < tilt_angle < 3.0:
            feedback = 0.0

        motor_vel = Twist()
        motor_vel.linear.x = -float(feedback)  # Left wheel speed
        motor_vel.linear.y = -float(feedback)   # Right wheel speed

        self.lqr_output.publish(motor_vel)

if __name__ == '__main__':
    rclpy.init()
    lqr_values = LQR()
    rclpy.spin(lqr_values)
    lqr_values.destroy_node()
    rclpy.shutdown()
