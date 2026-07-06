import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


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

    executable_path = os.path.join(
        get_package_prefix('rigol_demo'),
        'lib',
        'rigol_demo',
        'd435i_image_test',
    )

    d435i_test_process = ExecuteProcess(
        cmd=[
            executable_path,
            PythonExpression(["'--color_width=' + str(", LaunchConfiguration('color_width'), ")"]),
            PythonExpression(["'--color_height=' + str(", LaunchConfiguration('color_height'), ")"]),
            PythonExpression(["'--depth_width=' + str(", LaunchConfiguration('depth_width'), ")"]),
            PythonExpression(["'--depth_height=' + str(", LaunchConfiguration('depth_height'), ")"]),
            PythonExpression(["'--fps=' + str(", LaunchConfiguration('fps'), ")"]),
        ],
        output='screen',
    )

    return LaunchDescription([
        color_width_arg,
        color_height_arg,
        depth_width_arg,
        depth_height_arg,
        fps_arg,
        d435i_test_process,
    ])
