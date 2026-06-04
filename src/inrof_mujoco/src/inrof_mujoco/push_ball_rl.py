import gymnasium as gym
import numpy as np
import mujoco
from gymnasium import spaces
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "point.xml"

class PushBallEnv(gym.Env):
    def __init__(self, xml_path=DEFAULT_XML_PATH):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        self.step_count = 0
        self.max_steps = 500
        self.frame_skip = 5
