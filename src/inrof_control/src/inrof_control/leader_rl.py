import rclpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from swerm_msgs.action import LeaderPos
from stable_baselines3 import PPO
import numpy as np

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class LeaderRL(Node):
    def __init__(self):
        super().__init__("leader_rl")

        self.robot_num = 5
        self.robot_pos_ = np.full((2, self.robot_num), np.nan)
        self.robot_vel_ = np.full((2, self.robot_num), np.nan)
        self.odom_received_ = np.zeros(self.robot_num, dtype=bool)
        self.callback_group_ = ReentrantCallbackGroup()

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
                10,
                callback_group=self.callback_group_,
            )
            for i in range(self.robot_num)
        ]
        model_path = Path(get_package_share_directory("inrof_control")) / "models" / "ppo_point.zip"
        self.model_ = PPO.load(model_path, device="auto")

        self.rate_ = self.create_rate(20)
        self.leader_action_ = ActionServer(
            self,
            LeaderPos,
            'leader_pos',
            self.action_callback,
            callback_group=self.callback_group_
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

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        result = LeaderPos.Result()

        if not np.all(self.odom_received_):
            result.success = False
            result.msg = "waiting for all robot odometry"
            goal_handle.abort()
            return result
        
        goal = goal_handle.request
        target = np.array([
            goal.goal_pos.pose.position.x,
            goal.goal_pos.pose.position.y,
        ], dtype=np.float64)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result.success = False
                result.msg = "goal canceled"
                goal_handle.canceled()
                return result
            
            obs = [target[0], target[1]]
            for i in range(self.robot_num):
                obs.extend(self.robot_pos_[:, i])
                obs.extend(self.robot_vel_[:, i])

            obs = np.array(obs, dtype=np.float32)
            action, _ = self.model_.predict(obs, deterministic=True)

            swerm_center = self.robot_pos_.mean(axis=1)
            leader_xy = swerm_center + 2.0 * action

            txdata = Odometry()
            txdata.pose.pose.position.x = leader_xy[0]
            txdata.pose.pose.position.y = leader_xy[1]

            self.leader_publisher_.publish(txdata)
            
            distance = np.linalg.norm(target - swerm_center)
            if distance < 0.01:
                result.success = True
                result.msg = "goal succeeded"
                goal_handle.succeed()
                return result

            self.rate_.sleep()

        result.success = False
        result.msg = "rclpy shutdown"
        goal_handle.abort()
        return result

def main_leader(args=None):
    rclpy.init(args=args)

    node = LeaderRL()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
