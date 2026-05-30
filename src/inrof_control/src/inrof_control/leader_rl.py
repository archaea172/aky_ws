import rclpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from swerm_msgs.action import LeaderPos
from stable_baselines3 import PPO

class LeaderRL(Node):
    def __init__(self):
        super().__init__("leader_rl")
        self.leader_publisher_ = self.create_publisher(
            Odometry,
            "leader/odometry",
            10
        )
        self.robot_num = 5
        self.robot_odometry = [None] * self.robot_num
        self.odom_subscribers = [
            self.create_subscription(
                Odometry,
                f"robot_{i}/odometry",
                lambda msg, robot_id=i: self.odom_callback(robot_id, msg),
                10
            )
            for i in range(self.robot_num)
        ]
        model_path = Path(get_package_share_directory("inrof_control")) / "models" / "ppo_point.zip"
        self.model = PPO.load(model_path)

    def odom_callback(self, robot_id, msg):
        self.robot_odometry[robot_id] = msg
