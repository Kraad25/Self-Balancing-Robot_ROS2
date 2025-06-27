import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import EzPickle

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os
import xacro

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import DeleteEntity, SpawnEntity

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from data import *
import time

class SelfBalancingEnv(gym.Env, Node):
    def __init__(self, render_mode = None, maxStep=None):
        EzPickle.__init__(self, render_mode)

        # Action and Observation Space
        self.action_space = spaces.Box(Action["low"], Action["high"])
        self.observation_space = spaces.Box(Observation["low"], Observation["high"])

        # Training Variables
        self.max_step = maxStep
        self.current_step = 0
        self.game_over = False
        self.tilt = None
        self.tilt_rate = None
        self.imu_topic_ready = False
        self.contact_topic_ready = False
        self.prev_shaping = None


        # Subscriptions
        self.contact_detector_sub = self.create_subscription(Bool, '/base_contact', self._callback, 10)
        self.lqr_inp = self.create_subscription(Imu, '/lqr_input', self._lqrcallback, 10)

        # Publisher
        self.motor_output = self.create_publisher(Twist, '/motor_cmds', 10)

    def reset(self):
        self._reset_episode_variables()
        self._reset_gazebo_world()

        start_time = time.time()
        while not self.imu_ready or not self.contact_topic_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 5.0:  # timeout after 5 seconds
                self.get_logger().warn("Sensor data not received in time during reset.")
                break

        return np.array(self._get_state(), dtype=np.float32), {}

    def step(self, action):
        self.current_step += 1

        self._apply_action(action)

        state = self._get_state()
        reward = self._get_reward()
        terminated, truncated = self._check_done_conditions()
        
        return np.array(state, dtype=np.float32), float(reward), terminated, truncated, {}

    def render(self):
        pass

    def close(self):
        self.destroy_node()
        rclpy.shutdown()

    def _callback(self, msg : Bool):
        self.game_over = msg.data
        self.contact_topic_ready = True

    def _lqrcallback(self, msg : Imu):
        self.tilt = msg.orientation.y
        self.tilt_rate = msg.angular_velocity.y
        self.imu_topic_ready = True

    def _get_state(self):
        state = [self.tilt, self.tilt_rate]

        return state

    def _reset_episode_variables(self):  
        # Training Variables
        self.current_step = 0
        self.game_over = False
        self.tilt = None
        self.tilt_rate = None
        self.prev_shaping = None
        self.imu_topic_ready = False
        self.contact_topic_ready = False

    def _reset_gazebo_world(self):
        self._reset_simulation()
        self._delete_robot()
        self._create_robot()

    def _delete_robot(self):
        delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /delete_entity service...")
        
        delete_req = DeleteEntity.Request()
        delete_req.name = "SBR"

        future = delete_client.call_async(delete_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Successfully deleted robot.")
        else:
            self.get_logger().error("Failed to delete robot.")

    def _create_robot(self):
        spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /spawn_entity service...")

        pkg_path = get_package_share_directory("sbr_pjt")
        urdf_path = os.path.join(pkg_path, "urdf", "sbr.urdf")

        # Process the URDF using xacro
        doc = xacro.parse(open(urdf_path))
        xacro.process_doc(doc)
        robot_description = doc.toxml()

        spawn_req = SpawnEntity.Request()
        spawn_req.name = "SBR"
        spawn_req.xml = robot_description
        spawn_req.robot_namespace = ""
        spawn_req.reference_frame = "custom_world"
        spawn_req.initial_pose.position.x = 0.0
        spawn_req.initial_pose.position.y = 0.0
        spawn_req.initial_pose.position.z = 0.0

        future = spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Successfully spawned robot.")
        else:
            self.get_logger().error("Failed to spawn robot.")

    def _reset_simulation(self):
        reset_client = self.create_client(Empty, '/reset_simulation')
        if not reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Reset service unavailable, skipping reset.")
            return

        reset_req = Empty.Request()
        future = reset_client.call_async(reset_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Simulation successfully reset.")
        else:
            self.get_logger().warn("Simulation reset may have failed.")

    def _apply_action(self, action):
        motor_cmd = Twist()

        motor_cmd.linear.x = action[0] * MOTOR_SPEED
        motor_cmd.linear.y = action[0] * MOTOR_SPEED

        self.motor_output.publish(motor_cmd)

    def _get_reward(self):
        shaping = (10*self.tilt)

        reward = 0.0
        if self.prev_shaping is not None:
            reward = self.prev_shaping - shaping
        self.prev_shaping = shaping

        reward += 1.0 # Survival Bonus

        if self.max_step is not None and self.current_step >= self.max_step:
            reward += 5.0
        if self._check_fallen():
            reward -= 100.0

        return reward
    
    def _check_done_conditions(self):
        terminated = False
        truncated = False

        if self._check_fallen():
            terminated = True

        if self.max_step is not None and self.current_step >= self.max_step:
            truncated = True

        return terminated, truncated

