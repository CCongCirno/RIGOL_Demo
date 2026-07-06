import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # piper_no_gripper_moveit 的 demo.launch.py 会启动 move_group + rviz
    piper_moveit_share = get_package_share_directory('piper_no_gripper_moveit')
    moveit_demo_launch = os.path.join(
        piper_moveit_share, 'launch', 'demo.launch.py')

    # piper 单臂控制节点 launch
    piper_share = get_package_share_directory('piper')
    piper_single_launch = os.path.join(
        piper_share, 'launch', 'start_single_piper.launch.py')

    # ===== 目标位姿节点参数 =====
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value='/target_pose',
        description='接收目标位姿的话题(PoseStamped)')

    # ===== piper 单臂节点参数(透传给 start_single_piper.launch.py)=====
    can_port_arg = DeclareLaunchArgument(
        'can_port', default_value='can0',
        description='CAN 端口')
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable', default_value='true',
        description='自动使能机械臂')
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist', default_value='false',
        description='是否存在夹爪(无夹爪用 false)')
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple', default_value='2',
        description='夹爪速度倍率')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='日志级别 (debug, info, warn, error, fatal)')
    max_velocity_scaling_factor_arg = DeclareLaunchArgument(
        'max_velocity_scaling_factor', default_value='1.0',
        description='最大速度缩放因子')
    max_acceleration_scaling_factor_arg = DeclareLaunchArgument(
        'max_acceleration_scaling_factor', default_value='1.0',
        description='最大加速度缩放因子')

    moveit_pose_goal_node = Node(
        package='rigol_demo',
        executable='moveit_pose_goal_node',
        name='moveit_pose_goal_node',
        output='screen',
        parameters=[{
            'planning_group': 'arm',
            'end_effector_link': 'link6',
            'pose_topic': LaunchConfiguration('pose_topic'),
            'planning_time': 5.0,
            'goal_position_tolerance': 0.001,
            'goal_orientation_tolerance': 0.001,
            'plan_attempts': 5,
            'max_velocity_scaling_factor': LaunchConfiguration('max_velocity_scaling_factor'),
            'max_acceleration_scaling_factor': LaunchConfiguration('max_acceleration_scaling_factor'),
        }],
    )

    return LaunchDescription([
        # 目标位姿节点参数
        pose_topic_arg,
        max_velocity_scaling_factor_arg,
        max_acceleration_scaling_factor_arg,
        # piper 单臂节点参数
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        log_level_arg,
        # 启动 piper 单臂控制节点(硬件接口)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(piper_single_launch),
            launch_arguments={
                'can_port': LaunchConfiguration('can_port'),
                'auto_enable': LaunchConfiguration('auto_enable'),
                'gripper_exist': LaunchConfiguration('gripper_exist'),
                'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        ),
        # 启动 piper_no_gripper_moveit 的 move_group(以及 rviz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_demo_launch),
        ),
        moveit_pose_goal_node,
    ])
