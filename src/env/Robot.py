from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist
import rclpy

from tf_transformations import quaternion_from_euler
from data import *

class Robot:
    def __init__(self, env, training=True):
        self.env = env
        self.training = training
        self.motor_output = self.env.create_publisher(Twist, '/motor_cmds', 10)

    def reset_robot_pose(self):
        client = self._get_entity_client()
        request = self._create_reset_request()
        self._send_request(client, request)

    def apply_action(self, action):
        motor_cmd = Twist()

        motor_cmd.linear.x = action[0] * MOTOR_SPEED
        motor_cmd.linear.y = action[0] * MOTOR_SPEED

        self.motor_output.publish(motor_cmd)

    def _get_entity_client(self):
        client = self.env.create_client(SetEntityState, '/set_entity_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.env.get_logger().warn("Waiting for /set_entity_state service...")

        return client
    
    def _create_reset_request(self):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = "SBR"

        self._set_pose(request.state) # Pose (Position and Orientation)
        self._set_twist(request.state) # Twist (Velocity)

        return request
    
    def _send_request(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.env, future)

        if future.result() is None:
            self.env.get_logger().error("Failed to reset robot pose.")

    def _set_pose(self, state):
        state.pose.position.x = 0.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.2 

        tilt = self._apply_random_tilt()
        q = quaternion_from_euler(0.0, tilt, 0.0)
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]

    def _set_twist(self, state):
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

    def _apply_random_tilt(self):
        if self.training:
            return self.env.np_random.uniform(-INITIAL_TILT, INITIAL_TILT)
        return 0.0