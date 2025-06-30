#!/usr/bin/env python3

# Add this, (#!/usr/bin/env python3) at the top to prevent the error, Exec format error.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

class IMUReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.get_logger().info("IMU values are being recorded")

        self.subscription = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)
        self.publisher = self.create_publisher(Imu, '/rl_input', 10)

    def imu_callback(self, msg):
        imuValues = Imu()

        imuValues.angular_velocity.y = msg.angular_velocity.y
        imuValues.linear_acceleration.x = msg.linear_acceleration.x

        # Extract pitch (tilt) from IMU orientation
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        try:
            roll, pitch, yaw = euler_from_quaternion(q)
        except Exception as e:
            self.get_logger().error(f"Quaternion Conversion Error: {e}")
            return
        # Store tilt (pitch) in orientation.y for LQR to use directly
        imuValues.orientation.y = pitch
        
        self.publisher.publish(imuValues)
        
if __name__ == '__main__':
    rclpy.init()
    imu_subscriber = IMUReader()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()