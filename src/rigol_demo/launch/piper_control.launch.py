import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_port_arg = DeclareLaunchArgument(
        'can_port', default_value='can0', description='CAN port used by Piper')
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable', default_value='true', description='Automatically enable Piper')
    gripper_exist_arg = DeclareLaunchArgument(
        'gripper_exist', default_value='false', description='Whether Piper has a gripper')
    gripper_val_mutiple_arg = DeclareLaunchArgument(
        'gripper_val_mutiple', default_value='2', description='Piper gripper value multiple')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='warn', description='Piper driver log level')

    command_period_arg = DeclareLaunchArgument(
        'command_period', default_value='0.1', description='Command publish period in seconds')
    move_timeout_arg = DeclareLaunchArgument(
        'move_timeout', default_value='10.0', description='Timeout for each target pose in seconds')
    settle_time_arg = DeclareLaunchArgument(
        'settle_time', default_value='0.5', description='Settle time after each target is reached')
    arrival_ignore_time_arg = DeclareLaunchArgument(
        'arrival_ignore_time', default_value='0.3', description='Ignore stale arrived status for this duration')
    batch_mode_arg = DeclareLaunchArgument(
        'batch_mode', default_value='true',
        description='Batch mode: merge whole track into one Cartesian trajectory (no per-point pause)')
    batch_move_timeout_arg = DeclareLaunchArgument(
        'batch_move_timeout', default_value='60.0',
        description='Timeout for batch trajectory execution in seconds')
    gripper_arg = DeclareLaunchArgument(
        'gripper', default_value='0.0', description='Gripper command value')
    mode1_arg = DeclareLaunchArgument(
        'mode1', default_value='0', description='Piper PosCmd mode1 value')
    mode2_arg = DeclareLaunchArgument(
        'mode2', default_value='0', description='Piper PosCmd mode2 value')

    # ===== MoveIt 规划相关参数 =====
    # 规划模式按目标动态切换:home pose 用 OMPL 自由路径,目标点之间用 Pilz LIN 直线
    cartesian_speed_arg = DeclareLaunchArgument(
        'cartesian_speed', default_value='0.1',
        description='Cartesian speed (m/s) for linear motion between target points')
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='base_link',
        description='Reference frame for /target_pose')
    max_velocity_scaling_factor_arg = DeclareLaunchArgument(
        'max_velocity_scaling_factor', default_value='1.0',
        description='Max velocity scaling factor for MoveIt')
    max_acceleration_scaling_factor_arg = DeclareLaunchArgument(
        'max_acceleration_scaling_factor', default_value='1.0',
        description='Max acceleration scaling factor for MoveIt')
    planning_time_arg = DeclareLaunchArgument(
        'planning_time', default_value='5.0',
        description='MoveIt planning time in seconds')

    demo_msgs_lib = os.path.join(get_package_prefix('demo_msgs'), 'lib')
    piper_msgs_lib = os.path.join(get_package_prefix('piper_msgs'), 'lib')
    ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=[
            demo_msgs_lib,
            os.pathsep,
            piper_msgs_lib,
            os.pathsep,
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('piper'),
                'launch',
                'start_single_piper.launch.py',
            )
        ),
        launch_arguments={
            'can_port': LaunchConfiguration('can_port'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_exist': LaunchConfiguration('gripper_exist'),
            'gripper_val_mutiple': LaunchConfiguration('gripper_val_mutiple'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    # 启动 piper_no_gripper_moveit 的 move_group + rviz(demo.launch.py)
    piper_moveit_share = get_package_share_directory('piper_no_gripper_moveit')
    moveit_demo_launch = os.path.join(
        piper_moveit_share, 'launch', 'demo.launch.py')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_demo_launch),
    )

    # piper_control_node:订阅 /track_cmd,发布 /target_pose(PoseStamped)
    piper_control_node = Node(
        package='rigol_demo',
        executable='piper_control_node',
        name='piper_control_node',
        output='screen',
        parameters=[{
            'command_period': LaunchConfiguration('command_period'),
            'move_timeout': LaunchConfiguration('move_timeout'),
            'settle_time': LaunchConfiguration('settle_time'),
            'arrival_ignore_time': LaunchConfiguration('arrival_ignore_time'),
            'batch_mode': LaunchConfiguration('batch_mode'),
            'batch_move_timeout': LaunchConfiguration('batch_move_timeout'),
            'gripper': LaunchConfiguration('gripper'),
            'mode1': LaunchConfiguration('mode1'),
            'mode2': LaunchConfiguration('mode2'),
            'target_frame': LaunchConfiguration('target_frame'),
        }],
    )

    # moveit_pose_goal_node:订阅 /target_pose(PoseGoal),调用 MoveIt plan & execute
    # 规划模式由 PoseGoal.planner_mode 动态指定:0=OMPL(home),1=LIN(目标点之间)
    moveit_pose_goal_node = Node(
        package='rigol_demo',
        executable='moveit_pose_goal_node',
        name='moveit_pose_goal_node',
        output='screen',
        parameters=[{
            'planning_group': 'arm',
            'end_effector_link': 'link6',
            'pose_topic': '/target_pose',
            'planning_time': LaunchConfiguration('planning_time'),
            'goal_position_tolerance': 0.001,
            'goal_orientation_tolerance': 0.001,
            'plan_attempts': 5,
            'max_velocity_scaling_factor': LaunchConfiguration('max_velocity_scaling_factor'),
            'max_acceleration_scaling_factor': LaunchConfiguration('max_acceleration_scaling_factor'),
            'cartesian_speed': LaunchConfiguration('cartesian_speed'),
        }],
    )

    return LaunchDescription([
        can_port_arg,
        auto_enable_arg,
        gripper_exist_arg,
        gripper_val_mutiple_arg,
        log_level_arg,
        command_period_arg,
        move_timeout_arg,
        settle_time_arg,
        arrival_ignore_time_arg,
        batch_mode_arg,
        batch_move_timeout_arg,
        gripper_arg,
        mode1_arg,
        mode2_arg,
        cartesian_speed_arg,
        target_frame_arg,
        max_velocity_scaling_factor_arg,
        max_acceleration_scaling_factor_arg,
        planning_time_arg,
        ld_library_path,
        piper_launch,
        moveit_launch,
        piper_control_node,
        moveit_pose_goal_node,
    ])
