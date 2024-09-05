from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package share directory
    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    json_file_path = os.path.join(pkg_share, 'config', 'realsense-jo.json')

    # Define the RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        output='screen',
        parameters=[{
            'base_frame_id': 'base_link',
            'depth_module.profile': '1280x720x30',
            'pointcloud.enable': True,
            'pointcloud.pointcloud_qos': 'SENSOR_DATA',
            'pointcloud.frequency': 20,
            'json_file_path': json_file_path,

        }],
        remappings=[
            ('/camera/depth/color/points', '/points_raw')
        ]
    )

    # Return the LaunchDescription object
    return LaunchDescription([
        realsense_node
    ])
