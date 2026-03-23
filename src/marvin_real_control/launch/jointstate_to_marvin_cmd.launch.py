from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_joint_states_topic",
                default_value="/joint_states",
                description="Input JointState topic to bridge from",
            ),
            DeclareLaunchArgument(
                "left_cmd_topic",
                default_value="control/joint_cmd_A",
                description="Left arm marvin command topic",
            ),
            DeclareLaunchArgument(
                "right_cmd_topic",
                default_value="control/joint_cmd_B",
                description="Right arm marvin command topic",
            ),
            DeclareLaunchArgument(
                "publish_left",
                default_value="true",
                description="Whether to publish left arm commands",
            ),
            DeclareLaunchArgument(
                "publish_right",
                default_value="true",
                description="Whether to publish right arm commands",
            ),
            Node(
                package="marvin_real_control",
                executable="jointstate_to_marvin_cmd",
                name="jointstate_to_marvin_cmd",
                output="screen",
                parameters=[
                    {
                        "input_joint_states_topic": LaunchConfiguration("input_joint_states_topic"),
                        "left_cmd_topic": LaunchConfiguration("left_cmd_topic"),
                        "right_cmd_topic": LaunchConfiguration("right_cmd_topic"),
                        "publish_left": LaunchConfiguration("publish_left"),
                        "publish_right": LaunchConfiguration("publish_right"),
                    }
                ],
            ),
        ]
    )
