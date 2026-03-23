from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_side_arg = DeclareLaunchArgument(
        "arm_side",
        default_value="left",
        description="Arm side to monitor: left or right",
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/joint_states",
        description="Topic for joint states",
    )
    planned_path_topic_arg = DeclareLaunchArgument(
        "planned_path_topic",
        default_value="/display_planned_path",
        description="Topic for MoveIt planned trajectory",
    )

    demo_node = Node(
        package="marvin_real_control",
        executable="marvin_moveit_bridge_demo",
        name="marvin_moveit_bridge_demo",
        parameters=[
            {
                "arm_side": LaunchConfiguration("arm_side"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "planned_path_topic": LaunchConfiguration("planned_path_topic"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            arm_side_arg,
            joint_states_topic_arg,
            planned_path_topic_arg,
            demo_node,
        ]
    )
