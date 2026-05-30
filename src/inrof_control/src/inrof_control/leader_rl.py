import rclpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rclpy.action
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from swerm_msgs.action import LeaderPos
from stable_baselines3 import PPO
import numpy as np

class LeaderRL(Node):
    def __init__(self):
        super().__init__("leader_rl")

        self.robot_num = 5
        self.robot_pos_ = np.full((2, self.robot_num), np.nan)
        self.robot_vel_ = np.full((2, self.robot_num), np.nan)
        self.odom_received_ = np.zeros(self.robot_num, dtype=bool)

        self.leader_publisher_ = self.create_publisher(
            Odometry,
            "leader/odometry",
            10
        )
        self.odom_subscribers_ = [
            self.create_subscription(
                Odometry,
                f"robot_{i}/odometry",
                lambda msg, robot_id=i: self.odom_callback(robot_id, msg),
                10
            )
            for i in range(self.robot_num)
        ]
        model_path = Path(get_package_share_directory("inrof_control")) / "models" / "ppo_point.zip"
        self.model_ = PPO.load(model_path, device="auto")

        self.leader_action_ = ActionServer(
            self,
            LeaderPos,
            'leader',
            self.action_callback
        )

    def odom_callback(self, robot_id, msg):
        self.robot_pos_[:, robot_id] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.robot_vel_[:, robot_id] = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
        self.odom_received_[robot_id] = True

    def action_callback(self, goal_handle: rclpy.action.GoalResponse):
        self.get_logger().info('Executing goal...')
        if not np.all(self.odom_received_):
            self.get_logger().debug("Waiting for all robot odometry.")
            return
        
        goal = goal_handle.value()
        target_x = goal.goal_pos.pose.position.x
        target_y = goal.goal_pos.pose.position.y
        obs = [
            target_x, target_y,
            self.robot_pos_[0, 0], self.robot_pos_[1, 0], self.robot_vel_[0, 0], self.robot_vel_[1, 0],
            self.robot_pos_[0, 1], self.robot_pos_[1, 1], self.robot_vel_[0, 1], self.robot_vel_[1, 1],
            self.robot_pos_[0, 2], self.robot_pos_[1, 2], self.robot_vel_[0, 2], self.robot_vel_[1, 2],
            self.robot_pos_[0, 3], self.robot_pos_[1, 3], self.robot_vel_[0, 3], self.robot_vel_[1, 3]
        ]
        action, _ = self.model_.predict(obs)

        swerm_center = self.robot_pos_.mean(axis=1)
        leader_xy = swerm_center + 2.0 * action

        txdata = Odometry()
        txdata.pose.pose.position.x = leader_xy[0]
        txdata.pose.pose.position.y = leader_xy[1]

        self.leader_publisher_.publish(txdata)
