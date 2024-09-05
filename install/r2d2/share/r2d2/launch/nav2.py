
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    #Catrographer
    #map_file_path = 'maps/my_map.yaml'
    map_file_path = 'my_map.yaml'

    map_saver_params_file = 'map_saver.yaml'
    cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

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
    default_model_path = os.path.join(pkg_share, 'models/r2d2.urdf')
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekfultimate3.yaml')
    
    robot_name_in_urdf = 'atirobot'
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_configwithrealsense.rviz')
    map_save_config = os.path.join(pkg_share, 'params', map_saver_params_file)


    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace', default='')  # Assuming empty default namespace
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')

    #NAV2 SETUP
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    #static_map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    #static_map_path = os.path.join(nav2_bt_path, 'maps', 'my_map.yaml')
    static_map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')


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
  
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    
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
    
    declare_autostart_cmd = DeclareLaunchArgument(
      name='autostart', 
      default_value='false',
      description='Automatically startup the nav2 stack')
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
      name='default_bt_xml_filename',
      default_value=behavior_tree_xml_path,
      description='Full path to the behavior tree xml file to use')
    
  
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_share, 'maps', 'my_map.yaml'),
        description='Full path to map file to load')

    declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')
    
    declare_params_file_cmd = DeclareLaunchArgument(
      name='params_file',
      default_value=nav2_params_path,
      description='Full path to the ROS2 parameters file to use for all launched nodes')

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
    
    declare_slam_cmd = DeclareLaunchArgument(
      name='slam',
      default_value='False',
      description='Whether to run SLAM')



    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[robot_localization_file_path],
    )

  #  # Start imu_filter_madgwick node
  #   start_imu_filter_node_cmd = Node(
  #       package='imu_filter_madgwick',
  #       executable='imu_filter_madgwick_node',
  #       name='imu_filter_madgwick_node',
  #       output='screen',
  #       remappings=[
  #           ('imu/data_raw', '/imu')
  #       ],
  #       parameters=[{'use_mag': False, 'publish_tf': True, 'fixed_frame': 'camera_link', 'imu_frame': 'camera_imu_frame', 'gain': 0.4, 'world_frame': 'enu','sample_freq': 200.0}]
  #    )
    


    tflaunch_node = Node(
      ## Configure the TF of the robot to the origin of the map coordinates
      package='tf2_ros',
      executable='static_transform_publisher',
      output='screen',
      arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']
    )


    start_pcl_processing_node_cmd = Node(
        package='pcl_processing',
        executable='pcl_processing_node',
        name='pcl_processing_node',
        output='screen'
    )


    # Launch RViz
    start_rviz_cmd = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', rviz_config_file])


    start_ros2_navigation_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
      launch_arguments = {'namespace': namespace,
                          'use_namespace': use_namespace,
                          'map': map_yaml_file,
                          'slam':slam,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())


          # Laser Scan Merger Node
    laser_scan_filter_node = Node(
        package= 'laser_scan_filter_pkg',
        executable='laser_scan_filter_node',
        name='laser_scan_filter_node',
        output='screen',
        # remappings=[
        #     ('/scan_1', '/scan_1'),  # Ensuring it subscribes to the right topics
        #     ('/scan_2', '/scan_2'),
        #     ('/merged_scan', '/merged_scan')  # This is where the merged scan will be published
        # ]
    )


           # Laser Scan Merger Node
    laser_scan_merge_node = Node(
        package= 'laser_scan_merger_cam',
        executable='ros2_laser_scan_merger',
        name='ros2_laser_scan_merger',
        output='screen',
        # remappings=[
        #     ('/scan_1', '/scan_1'),  # Ensuring it subscribes to the right topics
        #     ('/scan_2', '/scan_2'),
        #     ('/merged_scan', '/merged_scan')  # This is where the merged scan will be published
        # ]
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
        arguments=['0', '0', '0', '0.73', '0', '0', '1', 'base_link', 'base_footrpint'],
        name='static_transform_publisher_base_node',
        output='screen'
    )

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(

      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher')
    
    start_syncnode = Node(
            package='my_sync_pkg',
            executable='sync_node',
            name='sync_node',
            output='screen',
        )



    # Launch description
    ld = LaunchDescription()


    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)



    ld.add_action(declare_model_path_cmd)
    #ld.add_action(declare_cartographer_config_dir_cmd)
    #ld.add_action(declare_configuration_basename_cmd)
    #ld.add_action(declare_resolution_cmd)
    #ld.add_action(declare_publish_period_sec_cmd)
    #ld.add_action(declare_use_joint_state_publisher_cmd)
    #ld.add_action(start_my_static_tf2_broadcaster_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd) 

    ld.add_action(declare_use_respawn_cmd)
 
    #ld.add_action(declare_use_rviz_cmd) 

    #Micro-rosagent
    # ld.add_action(start_micro_rSos_agent_node_cmd)
    # ld.add_action(start_rqt_robot_steering_node_cmd)
    # ld.add_action(start_hoverboard_bridge_node_cmd)
    #ld.add_action(start_wheel_odometry_publisher_node_cmd)

    #Camera nodes
    # ld.add_action(camera1_node)
    # ld.add_action(camera2_node)

    ld.add_action(start_pcl_processing_node_cmd)
    #ld.add_action(start_frame_changer_node)
    #ld.add_action(start_imu_complementary_filter_node)
    #ld.add_action(start_imu_filter_node_cmd)

    # ld.add_action(start_depthimage_to_laserscan_node_cmd_camera1)
    # ld.add_action(start_depthimage_to_laserscan_node_cmd_camera2)
    ld.add_action(laser_scan_filter_node)
    ld.add_action(laser_scan_merge_node)



    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_entity(tflaunch_node)
    ld.add_entity(start_static_tf_broadcaster_cmd)

    ld.add_action(start_joint_state_publisher_cmd)


    #ld.add_action(start_joint_state_publisher_gui_node)
    #ld.add_action(start_syncnode)
    ld.add_action(start_rviz_cmd)

    ld.add_action(start_ros2_navigation_cmd)




    return ld

