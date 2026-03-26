import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    gps_planning_share_dir = get_package_share_directory('gps_planning')

    # Define the path to the parameters file
    params_file = os.path.join(gps_planning_share_dir, 'config', 'f9r_roi_path.yaml')

    # Declare the node
    f9r_roi_path_node = Node(
        package='gps_planning',
        # The executable name is defined in CMakeLists.txt as 'f9r_roi_path'
        executable='f9r_roi_path',
        name='f9r_roi_path',
        output='screen',
        parameters=[
            params_file,
            {'roi_end_point_topic': '/gps/f9r_roi_end_velodyne'},
        ],
        remappings=[
            ('/csv_path', '/gps/csv_path'),
            ('/f9r_roi_path', '/gps/f9r_roi_path'),
            ('/f9r_roi_end', '/gps/f9r_roi_end'),
        ]
    )

    return LaunchDescription([
        f9r_roi_path_node
    ])
