import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    utm_tf_share_dir = get_package_share_directory('utm_tf')
    params_file = os.path.join(utm_tf_share_dir, 'config', 'tf_gps_csv.yaml')

    f9p_node = Node(
        package='utm_tf',
        executable='f9p_to_utm',
        name='f9p_to_utm',
        output='screen',
        remappings=[
            ('/f9p_utm', '/gps/f9p_utm'),
        ]
    )

    f9r_node = Node(
        package='utm_tf',
        executable='f9r_to_utm',
        name='f9r_to_utm',
        output='screen',
        remappings=[
            ('/f9r_utm', '/gps/f9r_utm'),
        ]
    )

    azimuth_node = Node(
        package='utm_tf',
        executable='azimuth_angle_calculator_node',
        name='azimuth_angle_calculator_node',
        output='screen',
        output_format='{line}',
        ros_arguments=['--log-level', 'warn'],
        remappings=[
            ('/azimuth_angle', '/gps/azimuth_angle'),
        ]
    )

    tf_gps_csv_node = Node(
        package='utm_tf',
        executable='tf_gps_csv_node',
        name='tf_gps_csv_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/f9r_utm', '/gps/f9r_utm'),
            ('/f9p_utm', '/gps/f9p_utm'),
            ('/azimuth_angle', '/gps/azimuth_angle'),
            ('/csv_path', '/gps/csv_path'),
            ('/azimuth_angle_text', '/gps/azimuth_angle_text'),
        ]
    )

    return LaunchDescription([
        f9p_node,
        f9r_node,
        azimuth_node,
        tf_gps_csv_node
    ])
