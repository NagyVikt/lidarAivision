
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir



def generate_launch_description():

    # Paths
    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    realsense_params_path = os.path.join(pkg_share, 'params', 'realsense_params.yaml')
    depthimagetolaserparams_path = os.path.join(pkg_share, 'params', 'depth_image_to_laserscan_params.yaml')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'models/atirobotbase.urdf')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekfultimate.yaml')
    
    robot_name_in_urdf = 'atirobot'
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/slamwithrealsense.rviz')

    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    namespace = LaunchConfiguration('namespace', default='')  # Assuming empty default namespace
    
 # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')

 declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
 
   declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
   
   declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

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

  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path,],
    remappings=[
        ('odometry/filtered', 'odom'),
    ],
  )


   # Start imu_filter_madgwick node
    start_imu_filter_node_cmd = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        remappings=[
            ('imu/data_raw', '/imu')
        ],
        parameters=[{'use_mag': False, 'publish_tf': True, 'fixed_frame': 'camera_link', 'imu_frame': 'camera_imu_frame'}]
     )


    start_pcl_processing_node_cmd = Node(
        package='pcl_processing',
        executable='pcl_processing_node',
        name='pcl_processing_node',
        output='screen'
    )
    
    # Micro ROS Agent Node
    start_micro_ros_agent_node_cmd = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0'],
        output='screen'
    )

    # Robot RQT Steering GUI Node
    start_rqt_robot_steering_node_cmd = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        arguments=['--force-discover']
    )

    # Hoverboard Bridge Node
    start_hoverboard_bridge_node_cmd = Node(
        package='hoverboard_bridge',
        executable='bridge_node',
        output='screen'
    )

    # Wheel Odometry Publisher Node
    start_wheel_odometry_publisher_node_cmd = Node(
        package='odometry_bridge',
        executable='wheel_odometry_publisher.py',
        output='screen'
    )

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

  tflaunch_node = Node(
    ## Configure the TF of the robot to the origin of the map coordinates
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
  )

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[ 'robot_description': Command(['xacro ', model])}],
    remappings=remappings,
    arguments=[default_model_path])


    # Launch description
    ld = LaunchDescription()
    
    ld.add_action(start_micro_ros_agent_node_cmd)
    ld.add_action(start_rqt_robot_steering_node_cmd)
    ld.add_action(start_hoverboard_bridge_node_cmd)
    ld.add_action(start_wheel_odometry_publisher_node_cmd)

    ld.add_action(start_realsense_node_cmd)
    ld.add_action(start_pcl_processing_node_cmd)
    ld.add_action(start_imu_filter_node_cmd)
    ld.add_action(start_depthimage_to_laserscan_node_cmd)
    
    ld.add_action(start_robot_localization_cmd)



    return ld

