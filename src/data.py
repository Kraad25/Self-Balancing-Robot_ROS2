import math
import numpy as np

# Action Space
Action = {
    # Motor speed percentage and Direction
    "low": np.array([-1]).astype(np.float32), 
    "high": np.array([1]).astype(np.float32)
}

# Observation Space
Observation = {
    # Observation includes,
    # Angle of tilt (Pitch in radians)
    # Speed of tilt (Angular velocity)
    "low": np.array([-math.pi/2, -15]).astype(np.float32),
    "high": np.array([math.pi/2, 15]).astype(np.float32),
}

# Motor Speed
MOTOR_SPEED = 5