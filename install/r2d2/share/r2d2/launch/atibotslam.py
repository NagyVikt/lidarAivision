# Author: Addison Sears-Collins
# Date: August 27, 2021
# Description: Launch a basic mobile robot URDF file using Rviz.
# https://automaticaddison.com

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


from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


def generate_launch_description():

  cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
  cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  cartographer_ros_prefix, 'configuration_files'))
  configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='rs_cartographer.lua')
  resolution = LaunchConfiguration('resolution', default='0.05')
  publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')


  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

  pkg_share = FindPackageShare(package='atirobotbase').find('atirobotbase')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'models/atirobotbase.urdf')
  robot_localization_file_path = os.path.join(pkg_share, 'config/ekfultimate.yaml')

  robot_name_in_urdf = 'atirobot'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/slamwithrealsense.rviz')

  depthimagetolaserparams_path = os.path.join(pkg_share, 'params', 'depth_image_to_laserscan_params.yaml')
  realsense_params_path = os.path.join(pkg_share, 'params', 'realsense_params.yaml')
  #gazebo world 
  world_file_name = 'basic_mobile_bot_world/smalltown.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)

  #NAV2 SETUP
  nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
  nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
  static_map_path = os.path.join(pkg_share, 'maps', 'factory_world/factory_world.yaml')
  nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
  nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
  behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')


   # Launch configuration variables specific to simulation
  autostart = LaunchConfiguration('autostart')
  default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
  headless = LaunchConfiguration('headless')
  map_yaml_file = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  model = LaunchConfiguration('model')
  namespace = LaunchConfiguration('namespace')
  slam = LaunchConfiguration('slam')
  use_namespace = LaunchConfiguration('use_namespace')
  params_file = LaunchConfiguration('params_file')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')

  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
  
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

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
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
  
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  # Specify the actions


 # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    arguments=[default_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

  # Declare the launch arguments

  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')

  declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')
  
  declare_bt_xml_cmd = DeclareLaunchArgument(
    name='default_bt_xml_filename',
    default_value=behavior_tree_xml_path,
    description='Full path to the behavior tree xml file to use')
  
  declare_map_yaml_cmd = DeclareLaunchArgument(
    name='map',
    default_value=static_map_path,
    description='Full path to map file to load')

  declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')

  declare_cartographer_config_dir_cmd = DeclareLaunchArgument(
    'cartographer_config_dir',
    default_value=cartographer_config_dir,
    description='Full path to config file to load')

  declare_configuration_basename_cmd = DeclareLaunchArgument(
    'configuration_basename',
    default_value=configuration_basename,
    description='Name of lua file for cartographer')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation (Gazebo) clock if true')

  declare_resolution_cmd = DeclareLaunchArgument(
    'resolution',
    default_value='0.05',
    description='Resolution of a grid cell in the published occupancy grid')

  declare_publish_period_sec_cmd = DeclareLaunchArgument(
    'publish_period_sec',
    default_value='1.0',
    description='OccupancyGrid publishing period')


  declare_model_path_cmd = DeclareLaunchArgument(
    name ='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
  
  declare_params_file_cmd = DeclareLaunchArgument(
    name='params_file',
    default_value=nav2_params_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')
  
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name = 'use_sim_time', default_value='true', 
    description='Use simulation (Gazebo) clock if true')
  
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name = 'world', 
    default_value=world_path,
    description='Full path to the world model file to load')

  qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
  )


  # Specify the actions

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

   # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}],
    remappings=[
        ('odometry/filtered', 'odom'),
    ],
  )


  start_pcl_processing_node_cmd = Node(
        package='pcl_processing',
        executable='pcl_processing_node',
        name='pcl_processing_node',
        output='screen'
    )

  start_realsense_node_cmd = Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    name='realsense2_camera_node',
    output='screen',
    parameters= [os.path.join(pkg_share, 'params', 'realsense_params.yaml'), {'use_sim_time': use_sim_time}]
  )

  depthimage_to_laserscan_nodee = ComposableNode(
    package='depthimage_to_laserscan',
    plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
    name='depthimage_to_laserscan',
    remappings=[
        ('depth', '/intel_realsense_r200_depth/image_raw'),
        ('depth_camera_info', '/intel_realsense_r200_depth/camera_info'),
    ],
  )

  start_depthimage_to_laserscan_node_cmd = Node(
    package='depthimage_to_laserscan',
    executable='depthimage_to_laserscan_node',
    name='depthimage_to_laserscan_node',
    output='screen',
    parameters=[depthimagetolaserparams_path, {'output_frame':'realsense_link'}],
    remappings=[('depth','/intel_realsense_r200_depth/depth/image_raw'),
                ('depth_camera_info', '/intel_realsense_r200_depth/depth/camera_info')],
  )

  container = ComposableNodeContainer(
    name='ComponentManager',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[depthimage_to_laserscan_nodee],
    output='screen',
    parameters=[ depthimagetolaserparams_path, {'use_sim_time': use_sim_time}]
  )

  tflaunch_node = Node(
    ## Configure the TF of the robot to the origin of the map coordinates
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
  )


  start_cartographer_node_cmd = Node(
    condition=IfCondition(slam),
    package='cartographer_ros',
    executable='cartographer_node',
    name='cartographer_node',
    output='screen',
    remappings=[('/points2', '/filtered_points')],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    arguments=['-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
               '-configuration_basename', LaunchConfiguration('configuration_basename')],
  )

  # Start Cartographer occupancy grid node
  start_cartographer_grid_node_cmd = Node(
    condition=IfCondition(slam),
    package='cartographer_ros',
    executable='occupancy_grid_node',
    name='occupancy_grid_node',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
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
    parameters=[{'use_sim_time': use_sim_time, 'use_mag': False, 'publish_tf': True, 'fixed_frame': 'camera_link', 'imu_frame': 'camera_imu_frame'}]
  )

  # Start the pointcloud_to_laserscan node
  start_pointcloud_to_laserscan_cmd = Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='pointcloud_to_laserscan',
    output='screen',
    remappings=[
        ('cloud_in', '/intel_realsense_r200_depth/points'),
    ],
    parameters= [os.path.join(pkg_share, 'params', 'pointcloud_node_params.yaml'), {'use_sim_time': use_sim_time}]

  )

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    remappings=remappings,
    arguments=[default_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])  

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_bt_xml_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_slam_cmd)

  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  ld.add_action(declare_cartographer_config_dir_cmd)
  ld.add_action(declare_configuration_basename_cmd)
  ld.add_action(declare_resolution_cmd)
  ld.add_action(declare_publish_period_sec_cmd)
 
  # Add any actions
  ld.add_action(start_cartographer_node_cmd)
  ld.add_action(start_cartographer_grid_node_cmd)
  ld.add_action(start_pcl_processing_node_cmd)
  #ld.add_action(start_realsense_node_cmd)
  # ld.add_action(start_imu_filter_node_cmd)
  #ld.add_action(start_pointcloud_to_laserscan_cmd)
  #ld.add_action(container)

  ld.add_entity(tflaunch_node)
  ld.add_action(start_depthimage_to_laserscan_node_cmd)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_robot_state_publisher_cmd)


  ld.add_action(start_rviz_cmd)

  return ld
