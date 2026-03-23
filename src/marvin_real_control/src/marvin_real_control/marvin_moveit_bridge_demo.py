#!/usr/bin/env python3

from typing import Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from moveit_msgs.msg import DisplayTrajectory
except ImportError:  # pragma: no cover - depends on runtime environment
    DisplayTrajectory = None


class MarvinMoveItBridgeDemo(Node):
    """Demo node: subscribe MoveIt related topics and summarize arm state."""

    def __init__(self) -> None:
        super().__init__("marvin_moveit_bridge_demo")

        self.declare_parameter("arm_side", "left")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("planned_path_topic", "/display_planned_path")
        self.declare_parameter("status_period_sec", 5.0)

        self.arm_side = str(self.get_parameter("arm_side").value).lower()
        if self.arm_side not in ("left", "right"):
            self.get_logger().warning("arm_side invalid, fallback to left")
            self.arm_side = "left"

        suffix = "L" if self.arm_side == "left" else "R"
        self.joint_names = [f"Joint{i}_{suffix}" for i in range(1, 8)]
        self.controller_trajectory_topic = f"/{self.arm_side}_arm_controller/joint_trajectory"

        self.current_joint_positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.latest_trajectory: Optional[Dict[str, object]] = None
        self.first_trajectory: Optional[Dict[str, object]] = None
        self.joint_states_received = 0
        self.trajectory_received = 0

        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        planned_path_topic = str(self.get_parameter("planned_path_topic").value)

        self.create_subscription(JointState, joint_states_topic, self.joint_states_callback, 50)
        self.create_subscription(
            JointTrajectory,
            self.controller_trajectory_topic,
            self.controller_trajectory_callback,
            10,
        )

        if DisplayTrajectory is not None:
            self.create_subscription(
                DisplayTrajectory,
                planned_path_topic,
                self.display_trajectory_callback,
                10,
            )
            self.get_logger().info(f"subscribed planned trajectory topic: {planned_path_topic}")
        else:
            self.get_logger().warning("moveit_msgs unavailable, skip /display_planned_path subscription")

        status_period = float(self.get_parameter("status_period_sec").value)
        if status_period <= 0.0:
            status_period = 5.0
        self.create_timer(status_period, self.status_callback)

        self.get_logger().info(f"demo started, arm_side={self.arm_side}")
        self.get_logger().info(f"listening joint_states: {joint_states_topic}")
        self.get_logger().info(f"listening controller trajectory: {self.controller_trajectory_topic}")
        self.get_logger().info(f"target joints: {self.joint_names}")

    def joint_states_callback(self, msg: JointState) -> None:
        matched = 0
        for i, name in enumerate(msg.name):
            if name in self.current_joint_positions and i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
                matched += 1

        if matched > 0:
            self.joint_states_received += 1

    def controller_trajectory_callback(self, msg: JointTrajectory) -> None:
        snapshot = self._snapshot_trajectory(
            source="controller_joint_trajectory",
            joint_names=msg.joint_names,
            points=msg.points,
        )
        if snapshot is not None:
            if self.first_trajectory is None:
                self.first_trajectory = snapshot
            self.latest_trajectory = snapshot
            self.trajectory_received += 1

    def display_trajectory_callback(self, msg) -> None:
        if len(msg.trajectory) == 0:
            return

        # Use latest RobotTrajectory in DisplayTrajectory
        robot_trajectory = msg.trajectory[-1]
        jt = robot_trajectory.joint_trajectory
        snapshot = self._snapshot_trajectory(
            source="display_planned_path",
            joint_names=jt.joint_names,
            points=jt.points,
        )
        if snapshot is not None:
            if self.first_trajectory is None:
                self.first_trajectory = snapshot
            self.latest_trajectory = snapshot
            self.trajectory_received += 1

    def _snapshot_trajectory(
        self, source: str, joint_names: List[str], points: List[object]
    ) -> Optional[Dict[str, object]]:
        if not points:
            return None

        index_map = [
            (target_name, joint_names.index(target_name))
            for target_name in self.joint_names
            if target_name in joint_names
        ]
        if not index_map:
            return None

        first_point = points[0]
        last_point = points[-1]
        first_pos = {}
        last_pos = {}
        for target_name, index in index_map:
            if index < len(first_point.positions):
                first_pos[target_name] = float(first_point.positions[index])
            if index < len(last_point.positions):
                last_pos[target_name] = float(last_point.positions[index])

        return {
            "source": source,
            "point_count": len(points),
            "joint_names": [name for name, _ in index_map],
            "first_point": first_pos,
            "last_point": last_pos,
        }

    def status_callback(self) -> None:
        if self.joint_states_received == 0:
            self.get_logger().warning("waiting joint_states...")
            return

        status = {
            "joint_states_count": self.joint_states_received,
            "trajectory_count": self.trajectory_received,
            "latest_trajectory_source": self.latest_trajectory["source"] if self.latest_trajectory else None,
        }
        self.get_logger().info(
            "status | joint_states=%s, trajectories=%s, latest_source=%s"
            % (
                status["joint_states_count"],
                status["trajectory_count"],
                status["latest_trajectory_source"],
            )
        )

    def _format_joint_map(self, joint_map: Optional[Dict[str, float]]) -> str:
        if not joint_map:
            return "-"
        return ", ".join(f"{name}={value:.3f}" for name, value in joint_map.items())

    def _trajectory_brief(self, traj: Optional[Dict[str, object]]) -> str:
        if traj is None:
            return "none"
        first_point = self._format_joint_map(traj.get("first_point"))
        last_point = self._format_joint_map(traj.get("last_point"))
        return (
            f"source={traj.get('source')}, points={traj.get('point_count')}\n"
            f"  start: {first_point}\n"
            f"  end:   {last_point}"
        )

    def print_summary(self) -> None:
        lines = [
            "========== MoveIt Bridge Demo Summary ==========",
            f"arm: {self.arm_side}",
            f"joint_states received: {self.joint_states_received}",
            f"trajectories received: {self.trajectory_received}",
            f"current joints: {self._format_joint_map(self.current_joint_positions)}",
            "first trajectory:",
            f"{self._trajectory_brief(self.first_trajectory)}",
            "latest trajectory:",
            f"{self._trajectory_brief(self.latest_trajectory)}",
            "================================================",
        ]
        self.get_logger().info("\n".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarvinMoveItBridgeDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.print_summary()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
