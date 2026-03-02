import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    utm_tf_share_dir = get_package_share_directory('utm_tf')

    # Define the path to the parameters file
    params_file = os.path.join(utm_tf_share_dir, 'config', 'tf_gps_csv.yaml')

    # Declare the node
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
        tf_gps_csv_node
    ])
