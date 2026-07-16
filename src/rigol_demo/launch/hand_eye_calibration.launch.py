import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 配置文件路径(默认使用 configs/hand_eye_calibration.yaml)
    default_config = os.path.join(
        get_package_share_directory('rigol_demo'),
        'configs',
        'hand_eye_calibration.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to hand-eye calibration YAML config file')

    hand_eye_node = Node(
        package='rigol_demo',
        executable='hand_eye_calibration_node',
        name='hand_eye_calibration_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            # 以下参数会覆盖配置文件中的同名项(留空则使用配置文件)
            'rgb_topic': '/camera/rgb/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
            'joint_state_topic': '/joint_states',
            'arm_status_topic': '/arm_status',
            'joint_ctrl_topic': '/joint_states',
            'base_frame': 'base_link',
            'ee_frame': 'link6',
            'camera_frame': 'camera_color_optical_frame',
            'board_width': 11,
            'board_height': 8,
            'square_size': 0.020,
            'use_custom_camera_matrix': True,
            'fx': 891.773183,
            'fy': 893.753426,
            'cx': 628.465114,
            'cy': 353.375401,
            'min_samples': 5,
            'max_samples': 20,
            'hand_eye_method': 1,
            'pose_wait_time': 5.0,
            'target_frame': 'base_link',
            'result_file': 'hand_eye_result.yaml',
            'robot_pose_file': 'robot_pose.yaml',
            'calibration_data_file': 'calibration_data.yaml',
            'window_name': 'Hand-Eye Calibration',
            'show_corners': True,
        }],
    )

    return LaunchDescription([
        config_file_arg,
        hand_eye_node,
    ])
