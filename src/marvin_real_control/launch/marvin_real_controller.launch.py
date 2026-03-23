from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arm_side_arg = DeclareLaunchArgument(
        'arm_side',
        default_value='left',
        description='控制的手臂(left/right)'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='控制频率(Hz)'
    )

    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states',
        description='关节状态话题'
    )

    planned_path_topic_arg = DeclareLaunchArgument(
        'planned_path_topic',
        default_value='/display_planned_path',
        description='MoveIt规划轨迹话题'
    )

    status_period_sec_arg = DeclareLaunchArgument(
        'status_period_sec',
        default_value='5.0',
        description='状态输出周期(秒)'
    )
    
    # 创建真实控制器节点
    real_controller_node = Node(
        package='marvin_real_control',
        executable='marvin_real_controller',
        name='marvin_real_controller',
        parameters=[{
            'arm_side': LaunchConfiguration('arm_side'),
            'control_rate': LaunchConfiguration('control_rate'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'planned_path_topic': LaunchConfiguration('planned_path_topic'),
            'status_period_sec': LaunchConfiguration('status_period_sec'),
        }],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        arm_side_arg,
        control_rate_arg,
        joint_states_topic_arg,
        planned_path_topic_arg,
        status_period_sec_arg,
        real_controller_node,
    ])
