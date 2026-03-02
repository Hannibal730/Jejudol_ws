import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the package's share directory
    utm_tf_share_dir = get_package_share_directory('utm_tf')

    # 1. f9p_to_utm node
    f9p_node = Node(
        package='utm_tf',
        executable='f9p_to_utm',
        name='f9p_to_utm',
        output='screen',
        remappings=[
            ('/f9p_utm', '/gps/f9p_utm'),
        ]
    )

    # 2. f9r_to_utm node
    f9r_node = Node(
        package='utm_tf',
        executable='f9r_to_utm',
        name='f9r_to_utm',
        output='screen',
        remappings=[
            ('/f9r_utm', '/gps/f9r_utm'),
        ]
    )

    # 3. azimuth_angle_calculator_node
    azimuth_node = Node(
        package='utm_tf',
        executable='azimuth_angle_calculator_node',
        name='azimuth_angle_calculator_node',
        output='screen',
        remappings=[
            ('/azimuth_angle', '/gps/azimuth_angle'),
        ]
    )

    # 4. Include tf_gps_csv.launch.py
    tf_gps_csv_launch_file = os.path.join(
        utm_tf_share_dir,
        'launch',
        'tf_gps_csv.launch.py'
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf_gps_csv_launch_file)
    )

    return LaunchDescription([
        f9p_node,
        f9r_node,
        azimuth_node,
        included_launch
    ])
