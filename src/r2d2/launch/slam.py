
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

    #Catrographer
    cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                    cartographer_ros_prefix, 'configuration_files'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                  default='rs_cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

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
    namespace = LaunchConfiguration('namespace', default='')  # Assuming empty default namespace

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
      name='gui',
      default_value='True',
      description='Flag to enable joint_state_publisher_gui')

 # Declare the launch arguments  
    declare_model_path_cmd = DeclareLaunchArgument(
      name='model', 
      default_value=default_model_path, 
      description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=default_rviz_config_path,
      description='Full path to the RVIZ config file to use')
  
   
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
      name='use_robot_state_pub',
      default_value='True',
      description='Whether to start the robot state publisher')

    declare_resolution_cmd = DeclareLaunchArgument(
    'resolution',
    default_value='0.05',
    description='Resolution of a grid cell in the published occupancy grid')

    declare_use_namespace_cmd = DeclareLaunchArgument(
      name='use_namespace',
      default_value='False',
      description='Whether to apply a namespace to the navigation stack')

    declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')

    declare_cartographer_config_dir_cmd = DeclareLaunchArgument(
      'cartographer_config_dir',
      default_value=cartographer_config_dir,
      description='Full path to config file to load')
    
    declare_publish_period_sec_cmd = DeclareLaunchArgument(
      'publish_period_sec',
      default_value='1.0',
      description='OccupancyGrid publishing period')
    
    declare_configuration_basename_cmd = DeclareLaunchArgument(
      'configuration_basename',
      default_value=configuration_basename,
      description='Name of lua file for cartographer')


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
        parameters=[depthimagetolaserparams_path, {'output_frame': 'camera_link'}],
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
         ('odom', '/odom'),
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
        parameters=[{'use_mag': False, 'publish_tf': True, 'fixed_frame': 'vcamera3_link', 'imu_frame': 'camera_imu_frame'}]
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
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file])




    start_cartographer_node_cmd = Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='cartographer_node',
      output='screen',
      remappings=[('/points2', '/filtered_points'),
      ('/odom', '/odometry/filtered')],

      arguments=['-configuration_directory', LaunchConfiguration('cartographer_config_dir'),
                '-configuration_basename', LaunchConfiguration('configuration_basename')],
    )

  # Start Cartographer occupancy grid node
    start_cartographer_grid_node_cmd = Node(
      package='cartographer_ros',
      executable='cartographer_occupancy_grid_node',
      name='occupancy_grid_node',
      output='screen',
      arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

      # Define the nav2_map_server map_saver_cli node execution
    start_map_saver_cli_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli'],
        output='screen'
    )



    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace=namespace,
      parameters=[{'robot_description': Command(['xacro ', model])}],
      remappings=remappings,
      arguments=[default_model_path])

      # Static Transform Broadcaster Node
    start_static_tf_broadcaster_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.73', '0', '0', '0', '1', 'base_footprint', 'base_link'],
        name='static_transform_publisher_node',
        output='screen'
    )

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(

      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher')

    # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      name='joint_state_publisher_gui')


    # Launch description
    ld = LaunchDescription()


    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_cartographer_config_dir_cmd)
    ld.add_action(declare_configuration_basename_cmd)

    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_sec_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)


    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    #ld.add_action(declare_use_rviz_cmd) 

    #Micro-rosagent
    # ld.add_action(start_micro_rSos_agent_node_cmd)
    # ld.add_action(start_rqt_robot_steering_node_cmd)
    # ld.add_action(start_hoverboard_bridge_node_cmd)
    ld.add_action(start_wheel_odometry_publisher_node_cmd)

    #Camera nodes
    ld.add_action(start_realsense_node_cmd)
    ld.add_action(start_pcl_processing_node_cmd)
    ld.add_action(start_imu_filter_node_cmd)
    ld.add_action(start_depthimage_to_laserscan_node_cmd)

    #Robot localization
    #ld.add_entity(tflaunch_node)
    ld.add_action(start_static_tf_broadcaster_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_joint_state_publisher_cmd)


    #ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_cartographer_node_cmd)
    ld.add_action(start_cartographer_grid_node_cmd)
    #ld.add_action(start_map_saver_cli_cmd)

    ld.add_action(start_rviz_cmd)



    return ld

