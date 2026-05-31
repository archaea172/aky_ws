import gymnasium as gym
import numpy as np
import mujoco
from mujoco import mjx
from gymnasium import spaces
import inrof_swerm
from pathlib import Path
import jax
import jax.numpy as jnp

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML_PATH = PROJECT_ROOT / "models" / "point.xml"

class MJXPointEnv(gym.Env):
    def __init__(self, xml_path=DEFAULT_XML_PATH, robot_num=5):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.mjx_model = mjx.put_model(self.model)
        self.data = mjx.make_data(self.mjx_model)
        
        self.robot_num = robot_num
        self.step_count = 0
        self.max_steps = 600
        self.frame_skip = 5
        self.target_position = np.array([3.0, 3.0], dtype=np.float64)
        self.success_threshold = 0.25
        self.leader_offset_scale = 2.0
        
        self.vx_ids = np.array([
            self.model.actuator(f"robot_{i}_vx").id
            for i in range(robot_num)
        ])
        self.vy_ids = np.array([
            self.model.actuator(f"robot_{i}_vy").id
            for i in range(robot_num)
        ])

        self.robot_x_qpos_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qpos_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_x_qvel_ids = np.empty(robot_num, dtype=np.intp)
        self.robot_y_qvel_ids = np.empty(robot_num, dtype=np.intp)
        
        for i in range(robot_num):
            x_joint_id = self.model.joint(f"robot_{i}_x").id
            y_joint_id = self.model.joint(f"robot_{i}_y").id
            
            self.robot_x_qpos_ids[i] = self.model.jnt_qposadr[x_joint_id]
            self.robot_y_qpos_ids[i] = self.model.jnt_qposadr[y_joint_id]
            self.robot_x_qvel_ids[i] = self.model.jnt_dofadr[x_joint_id]
            self.robot_y_qvel_ids[i] = self.model.jnt_dofadr[y_joint_id]
            
        self._robot_pos_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._robot_vel_matrix = np.empty((2, robot_num), dtype=np.float64)
        self._leader_pos = np.empty(2, dtype=np.float64)
        self._prev_robot_vel_matrix = np.zeros((2, self.robot_num), dtype=np.float64)

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.robot_num * 4 + 2,),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )
        
        boid_params = inrof_swerm.BoidPrams()
        boid_params.boid_num = robot_num
        boid_params.max_vel = 0.2
        boid_params.Ir = 100.0
        boid_params.Ir_min = 0.01
        boid_params.k_separation = 1.0
        boid_params.k_alignment = 1.1
        boid_params.k_gravity = 1.0
        boid_params.k_wall = 0.0

        self.follower_core = inrof_swerm.FollowerCore(boid_params, 0.5)

