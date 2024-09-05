import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Define the package directory
    pkg_share = get_package_share_directory('r2d2')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/slamwithrealsenselidar.rviz')
    # LIDAR Configuration
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port_right = LaunchConfiguration('serial_port_right', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id_right = LaunchConfiguration('frame_id_right', default='lidarright')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode_right = LaunchConfiguration('scan_mode_right', default='Standard')
    topic_name_right = LaunchConfiguration('topic_name_right', default='scan_right')

    # LIDAR Declarations
    declare_channel_type = DeclareLaunchArgument(
        'channel_type',
        default_value=channel_type,
        description='Specifying channel type of lidar'
    )
    declare_serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying USB port baudrate to connected lidar'
    )
    declare_inverted = DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data'
    )
    declare_angle_compensate = DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether or not to enable angle compensation of scan data'
    )
    declare_frame_id_right = DeclareLaunchArgument(
        'frame_id_right',
        default_value=frame_id_right,
        description='Specifying frame_id of lidar'
    )
    declare_scan_mode_right = DeclareLaunchArgument(
        'scan_mode_right',
        default_value=scan_mode_right,
        description='Specifying scan mode of lidar'
    )
    declare_serial_port_right = DeclareLaunchArgument(
        'serial_port_right',
        default_value=serial_port_right,
        description='Specifying USB port to connected right lidar'
    )
    declare_topic_name_right = DeclareLaunchArgument(
        'topic_name_right',
        default_value=topic_name_right,
        description='Specifying topic name of right lidar scan data'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=default_rviz_config_path,
      description='Full path to the RVIZ config file to use')


    # Start RPLIDAR Node
    start_rplidar_ros_right = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node_right',
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port_right,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id_right,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'topic_name': topic_name_right,
            'scan_mode': scan_mode_right
        }],
        output='screen'
    )
    
    
        # Launch RViz
    start_rviz_cmd = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file])



    # Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_channel_type)
    ld.add_action(declare_serial_baudrate)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_inverted)
    ld.add_action(declare_angle_compensate)
    ld.add_action(declare_frame_id_right)
    ld.add_action(declare_scan_mode_right)
    ld.add_action(declare_serial_port_right)
    ld.add_action(declare_topic_name_right)
    ld.add_action(start_rplidar_ros_right)


    ld.add_action(start_rviz_cmd)
    return ld

