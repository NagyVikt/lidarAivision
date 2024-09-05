import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Paths
    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    realsense_params_path = os.path.join(pkg_share, 'params', 'realsense_params.yaml')
    depthimagetolaserparams_path = os.path.join(pkg_share, 'params', 'depth_image_to_laserscan_params.yaml')

    # Nodes related to RealSense, point cloud, and depth image processing
    start_realsense_node_cmd = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',
        parameters=[realsense_params_path]
    )

    start_depthimage_to_laserscan_node_cmd = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        output='screen',
        parameters=[depthimagetolaserparams_path, {'output_frame': 'realsense_link'}],
        remappings=[
            ('depth', '/depth/image_rect_raw'),
            ('depth_camera_info', '/depth/camera_info')
        ]
    )

    start_pcl_processing_node_cmd = Node(
        package='pcl_processing',
        executable='pcl_processing_node',
        name='pcl_processing_node',
        output='screen'
    )

    # Launch description
    ld = LaunchDescription()

    ld.add_action(start_realsense_node_cmd)
    ld.add_action(start_pcl_processing_node_cmd)
    ld.add_action(start_depthimage_to_laserscan_node_cmd)

    return ld

