import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import EzPickle

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from data import *
from contact_detector import ContactDetector
from Robot import Robot

import time

class SelfBalancingEnv(gym.Env, Node):
    def __init__(self, render_mode = None, maxStep=None, training=True):
        EzPickle.__init__(self, render_mode)
        Node.__init__(self, 'self_balancing_env')

        # Action and Observation Space
        self.action_space = spaces.Box(Action["low"], Action["high"])
        self.observation_space = spaces.Box(Observation["low"], Observation["high"])

        # Training Variables
        self.max_step = maxStep
        self.training = training
        self.current_step = 0
        
        # Observation Variables
        self.tilt = None
        self.tilt_rate = None
        
        # Reward Variables
        self.prev_shaping = None
        
        # Object Creations
        self.contact_detector = ContactDetector(self)
        self.robot = Robot(self, self.training)

        # Subscriptions
        self.rl_inp = self.create_subscription(Imu, '/rl_input', self._callback, 10)

    def reset(self, seed=None):
        super().reset(seed=seed)

        # To prevent false positive while resetting the robot pose
        self.contact_detector.set_enable_detection(False)

        self.robot.reset_robot_pose()
        self._reset_episode_variables()
        self._ensure_imu_ready()

        self.contact_detector.set_enable_detection(True)

        return np.array(self._get_state(), dtype=np.float32), {}

    def step(self, action):
        self.current_step += 1

        self.robot.apply_action(action)
        rclpy.spin_once(self, timeout_sec=0.02)

        state = self._get_state()
        reward = self._get_reward()
        terminated, truncated = self._check_done_conditions()
        
        return np.array(state, dtype=np.float32), float(reward), terminated, truncated, {}

    def render(self):
        pass

    def close(self):
        self.destroy_node()
        rclpy.shutdown()

    def _callback(self, msg : Imu):
        self.tilt = msg.orientation.y
        self.tilt_rate = msg.angular_velocity.y

    def _get_state(self):
        state = [self.tilt, self.tilt_rate]
        return state

    def _reset_episode_variables(self):  
        self.current_step = 0
        self.tilt = None
        self.tilt_rate = None
        self.prev_shaping = None
        self.contact_detector.reset()

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
    
    def _check_fallen(self):
        return self.contact_detector.has_fallen()

    def _ensure_imu_ready(self, timeout_sec=5.0):
        # To prevent the imu values being None in _get_state() we wait to get the value after resetting it.
        deadline  = time.time() + timeout_sec
        while (self.tilt is None or self.tilt_rate is None) and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.tilt is None or self.tilt_rate is None:
            self.get_logger().warn("Timeout waiting for IMU data.")