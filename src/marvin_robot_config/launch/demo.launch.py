from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import SetRemap


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("marvin_robot", package_name="marvin_robot_config").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)
    ld.entities.insert(0, SetRemap(src="/joint_states", dst="/moveit_joint_states"))
    return ld
