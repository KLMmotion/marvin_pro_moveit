#!/usr/bin/env python3

from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robotnode_marvin_msgs.msg import Jointcmd


class JointStateToMarvinCmd(Node):
    def __init__(self) -> None:
        super().__init__("jointstate_to_marvin_cmd")

        self.declare_parameter("input_joint_states_topic", "/joint_states")
        self.declare_parameter("left_cmd_topic", "control/joint_cmd_A")
        self.declare_parameter("right_cmd_topic", "control/joint_cmd_B")
        self.declare_parameter("publish_left", True)
        self.declare_parameter("publish_right", True)
        self.declare_parameter(
            "left_joint_names",
            [f"Joint{i}_L" for i in range(1, 8)],
        )
        self.declare_parameter(
            "right_joint_names",
            [f"Joint{i}_R" for i in range(1, 8)],
        )

        input_topic = str(self.get_parameter("input_joint_states_topic").value)
        left_cmd_topic = str(self.get_parameter("left_cmd_topic").value)
        right_cmd_topic = str(self.get_parameter("right_cmd_topic").value)
        self.publish_left = bool(self.get_parameter("publish_left").value)
        self.publish_right = bool(self.get_parameter("publish_right").value)
        self.left_joint_names = list(self.get_parameter("left_joint_names").value)
        self.right_joint_names = list(self.get_parameter("right_joint_names").value)

        self.left_pub = self.create_publisher(Jointcmd, left_cmd_topic, 20)
        self.right_pub = self.create_publisher(Jointcmd, right_cmd_topic, 20)
        self.create_subscription(JointState, input_topic, self.joint_state_callback, 50)

        self.missing_warn_count = 0
        self.forward_count = 0
        self.get_logger().info(f"listening joint states: {input_topic}")
        self.get_logger().info(f"publishing left cmds: {left_cmd_topic}")
        self.get_logger().info(f"publishing right cmds: {right_cmd_topic}")

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_position: Dict[str, float] = {}
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                name_to_position[name] = float(msg.position[idx])

        if self.publish_left:
            self._publish_arm_cmd(self.left_pub, self.left_joint_names, name_to_position, "left")
        if self.publish_right:
            self._publish_arm_cmd(self.right_pub, self.right_joint_names, name_to_position, "right")

    def _publish_arm_cmd(
        self,
        publisher,
        joint_names: List[str],
        name_to_position: Dict[str, float],
        arm_label: str,
    ) -> None:
        missing = [name for name in joint_names if name not in name_to_position]
        if missing:
            self.missing_warn_count += 1
            if self.missing_warn_count <= 5 or self.missing_warn_count % 50 == 0:
                self.get_logger().warning(
                    f"{arm_label} arm joints missing from input JointState: {missing}"
                )
            return

        cmd = Jointcmd()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.positions = [name_to_position[name] for name in joint_names]
        publisher.publish(cmd)
        self.forward_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateToMarvinCmd()
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
