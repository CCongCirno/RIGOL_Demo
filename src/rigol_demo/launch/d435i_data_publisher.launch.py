import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_width_arg = DeclareLaunchArgument(
        'color_width', default_value='1280', description='RGB image width')
    color_height_arg = DeclareLaunchArgument(
        'color_height', default_value='720', description='RGB image height')
    depth_width_arg = DeclareLaunchArgument(
        'depth_width', default_value='1280', description='Depth image width')
    depth_height_arg = DeclareLaunchArgument(
        'depth_height', default_value='720', description='Depth image height')
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30', description='Camera frame rate')

    d435i_publisher_node = Node(
        package='rigol_demo',
        executable='d435i_data_publisher_node',
        name='d435i_data_publisher_node',
        output='screen',
        parameters=[{
            'color_width': LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
            'depth_width': LaunchConfiguration('depth_width'),
            'depth_height': LaunchConfiguration('depth_height'),
            'fps': LaunchConfiguration('fps'),
        }],
    )

    return LaunchDescription([
        color_width_arg,
        color_height_arg,
        depth_width_arg,
        depth_height_arg,
        fps_arg,
        d435i_publisher_node,
    ])
