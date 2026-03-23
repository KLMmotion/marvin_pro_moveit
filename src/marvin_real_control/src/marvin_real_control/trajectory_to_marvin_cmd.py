#!/usr/bin/env python3

import time
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robotnode_marvin_msgs.msg import Jointcmd


class TrajectoryToMarvinCmd(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_to_marvin_cmd")

        self.declare_parameter("left_action_name", "/left_arm_controller/follow_joint_trajectory")
        self.declare_parameter("right_action_name", "/right_arm_controller/follow_joint_trajectory")
        self.declare_parameter("left_cmd_topic", "control/joint_cmd_A")
        self.declare_parameter("right_cmd_topic", "control/joint_cmd_B")
        self.declare_parameter("left_joint_names", [f"Joint{i}_L" for i in range(1, 8)])
        self.declare_parameter("right_joint_names", [f"Joint{i}_R" for i in range(1, 8)])

        self.left_joint_names = list(self.get_parameter("left_joint_names").value)
        self.right_joint_names = list(self.get_parameter("right_joint_names").value)

        left_action_name = str(self.get_parameter("left_action_name").value)
        right_action_name = str(self.get_parameter("right_action_name").value)
        left_cmd_topic = str(self.get_parameter("left_cmd_topic").value)
        right_cmd_topic = str(self.get_parameter("right_cmd_topic").value)

        self.left_pub = self.create_publisher(Jointcmd, left_cmd_topic, 20)
        self.right_pub = self.create_publisher(Jointcmd, right_cmd_topic, 20)

        self.left_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            left_action_name,
            execute_callback=self.execute_left,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.right_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            right_action_name,
            execute_callback=self.execute_right,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(f"left action server: {left_action_name}")
        self.get_logger().info(f"right action server: {right_action_name}")
        self.get_logger().info(f"publishing left cmd: {left_cmd_topic}")
        self.get_logger().info(f"publishing right cmd: {right_cmd_topic}")

    def goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> int:
        if len(goal_request.trajectory.points) == 0:
            self.get_logger().warning("reject empty trajectory goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> int:
        return CancelResponse.ACCEPT

    def execute_left(self, goal_handle) -> FollowJointTrajectory.Result:
        return self._execute(goal_handle, self.left_joint_names, self.left_pub, "left")

    def execute_right(self, goal_handle) -> FollowJointTrajectory.Result:
        return self._execute(goal_handle, self.right_joint_names, self.right_pub, "right")

    def _execute(
        self,
        goal_handle,
        target_joint_names: List[str],
        publisher,
        arm_label: str,
    ) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory
        joint_index: Dict[str, int] = {name: idx for idx, name in enumerate(traj.joint_names)}

        for joint_name in target_joint_names:
            if joint_name not in joint_index:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = f"{arm_label} arm missing joint {joint_name}"
                return result

        start = self.now_sec()
        for point in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            target_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            while self.now_sec() - start < target_time:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                time.sleep(0.001)

            cmd = Jointcmd()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.positions = [float(point.positions[joint_index[name]]) for name in target_joint_names]
            publisher.publish(cmd)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(target_joint_names)
            feedback.desired = point
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def now_sec(self) -> float:
        now = self.get_clock().now().to_msg()
        return float(now.sec) + float(now.nanosec) * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryToMarvinCmd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
