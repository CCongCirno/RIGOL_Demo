import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 配置文件路径
    default_config = os.path.join(
        get_package_share_directory('rigol_demo'), 'configs', 'trajectory_draw.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to trajectory_draw.yaml config file')

    # 颜色梯度分割可视化调参开关 (命令行可覆盖)
    debug_seg_arg = DeclareLaunchArgument(
        'enable_segmentation_debug', default_value='false',
        description='Enable segmentation debug visualization for tuning')
    # 确认轨迹时是否显示 "Reprojected Trajectory on RGB" 和 "Screen Warped to Canvas" 窗口
    show_reproject_arg = DeclareLaunchArgument(
        'show_reproject_windows', default_value='false',
        description='Show reprojected trajectory on RGB and screen warped to canvas windows')

    trajectory_draw_node = Node(
        package='rigol_demo',
        executable='trajectory_draw_node',
        name='trajectory_draw_node',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'enable_segmentation_debug': LaunchConfiguration('enable_segmentation_debug')},
            {'show_reproject_windows': LaunchConfiguration('show_reproject_windows')},
        ],
    )

    return LaunchDescription([
        config_file_arg,
        debug_seg_arg,
        show_reproject_arg,
        trajectory_draw_node,
    ])
