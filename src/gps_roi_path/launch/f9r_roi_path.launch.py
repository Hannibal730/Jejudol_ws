import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    gps_roi_path_share_dir = get_package_share_directory('gps_roi_path')

    # Define the path to the parameters file
    params_file = os.path.join(gps_roi_path_share_dir, 'config', 'f9r_roi_path.yaml')

    # Declare the node
    f9r_roi_path_node = Node(
        package='gps_roi_path',
        # The executable name is defined in CMakeLists.txt as 'f9r_roi_path'
        executable='f9r_roi_path',
        name='f9r_roi_path',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        f9r_roi_path_node
    ])
