import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    gps_roi_path_share_dir = get_package_share_directory('gps_roi_path')

    # Define the path to the parameters file
    params_file = os.path.join(gps_roi_path_share_dir, 'config', 'pure_pursuit.yaml')

    # Declare the node
    pure_pursuit_node = Node(
        package='gps_roi_path',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        pure_pursuit_node
    ])
