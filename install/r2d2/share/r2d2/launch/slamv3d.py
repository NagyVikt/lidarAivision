
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir



def generate_launch_description():
    #Catrographer
    map_saver_params_file = 'map_saver.yaml'
    cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                    cartographer_ros_prefix, 'configuration_files'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                  default='rs_cartographer3d.lua')
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
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/slamwithrealsense4.rviz')
    map_save_config = os.path.join(pkg_share, 'params', map_saver_params_file)

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


    camera1_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',

        parameters=[ {'serial_no': '213322072999',
        'base_frame_id': 'base_link',
        'enable_acel' : True,
        'enable_gyro': True,
        'enable_color': True,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_sync': True,
        'publish_odom_tf': True,
        'publish_tf': True,
        'unite_imu_method':2,
        'use_sim_time': False,
        'pointcloud.allow_no_texture_points': False,
                    'pointcloud.enable': True,
                    'pointcloud.filter_magnitude': 1,
                    'pointcloud.frames_queue_size': 16,
                    'pointcloud.ordered_pc': False,
                    'pointcloud.pointcloud_qos': 'DEFAULT',
                    'pointcloud.stream_filter': 2,              
                    'pointcloud.stream_format_filter': 0,
                    'pointcloud.stream_index_filter': 0}]


    )

    camera2_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node2',
        namespace='camera2',
        output='screen',

        parameters=[ {'serial_no': '239722073171',
        'base_frame_id': 'base_link',
        'enable_acel' : False,
        'enable_gyro': False,
        'enable_color': True,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_sync': True,
        'publish_odom_tf': False,
        'publish_tf': True,
        'use_sim_time': False,
        'pointcloud.enable': False}]

        )


    start_depthimage_to_laserscan_node_cmd_camera1 = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node_camera1',
        output='screen',
        parameters=[depthimagetolaserparams_path, {'output_frame': 'camera_link'}],
        remappings=[
            ('depth', '/depth/image_rect_raw'),
            ('depth_camera_info', '/depth/camera_info'),
            ('scan', '/scan1_dirt')  # Remapping the scan topic to /scan_2

        ]
    )

    start_depthimage_to_laserscan_node_cmd_camera2 = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node_camera2',
        output='screen',
        parameters=[depthimagetolaserparams_path, {'output_frame': 'camera2_link'}],
        remappings=[
            ('depth', '/camera2/depth/image_rect_raw'),
            ('depth_camera_info', '/camera2/depth/camera_info'),
            ('scan', '/scan2_dirt')  # Remapping the scan topic to /scan_2

        ]
    )
   


    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[robot_localization_file_path,],
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
        parameters=[{'use_mag': False, 'publish_tf': True, 'fixed_frame': 'camera_link', 'imu_frame': 'camera_imu_frame', 'gain': 0.8,'orientation_stddev': 0.1, 'world_frame': 'enu','sample_freq': 200.0}]
     )
    
    delayed_start_madgwick_filter = TimerAction(
          period=1.0,  # Delay duration in seconds
          actions=[
              start_imu_filter_node_cmd
  
          ]
      )


    start_imu_complementary_filter_node = Node(
          package='imu_complementary_filter',
          executable='complementary_filter_node',
          name='complementary_filter_node',
          output='screen',
          parameters=[{
              'use_mag': False, 
              'gain_acc': 0.01, 
              'gain_mag': 0.01, 
              'stateless': True,
              'fixed_frame': 'camera_link', # Ensure this is set to the appropriate frame
              'imu_frame': 'camera_imu_frame'     # Ensure this is set to the appropriate frame
          }],
          remappings=[
              ('imu/data_raw', '/imu'),
              ('imu/data', '/imu/data_filtered')
          ]
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
      remappings=[
      ('odom', '/odometry/filtered'),
      ('imu', '/imu/data'),
],

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


    delayed_start_cartographer_grid_node_cmd = TimerAction(
          period=1.0,  # Delay duration in seconds
          actions=[
              start_cartographer_grid_node_cmd
          ]
      )
    
    delayed_start_cartographer_node_cmd = TimerAction(
          period=1.0,  # Delay duration in seconds
          actions=[
              start_cartographer_node_cmd
          ]
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

    #       # Static Transform Broadcaster Node
    # start_static_tf_camera_cmd = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0.267', '0.062', '-1.44', '0', '2', '0.8254', '0.5645', 'base_link', 'camera2_link'],
    #     name='static_transform_publisher_camera_node',
    #     output='screen'
    # )

    start_odom_z_filter = Node(
        package='zero_z_odometry',
        executable='zero_z_odometry_node',
        name='zero_z_odometry_node',
        output='screen',
    )


    start_imu_filter_custom_node = Node(
        package='imu_filter_node',
        executable='imu_filter',
        name='imu_filter_custom',
        output='screen',
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
    #ld.add_action(start_my_static_tf2_broadcaster_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    #ld.add_action(declare_use_rviz_cmd) 


    ld.add_action(start_wheel_odometry_publisher_node_cmd)

    #Camera nodes
    ld.add_action(camera1_node)
    ld.add_action(camera2_node)

    ld.add_action(start_pcl_processing_node_cmd)
    #ld.add_action(start_frame_changer_node)
    #ld.add_action(start_imu_complementary_filter_node)
    ld.add_action(start_imu_filter_node_cmd)

    ld.add_action(start_depthimage_to_laserscan_node_cmd_camera1)
    ld.add_action(start_depthimage_to_laserscan_node_cmd_camera2)
    ld.add_action(laser_scan_filter_node)
    ld.add_action(laser_scan_merge_node)


    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)

    ld.add_action(start_joint_state_publisher_cmd)
    #ld.add_action(start_odom_z_filter)
    #ld.add_action(start_imu_filter_custom_node)


    #ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_cartographer_node_cmd)
    ld.add_action(start_cartographer_grid_node_cmd)


    ld.add_action(start_rviz_cmd)


    return ld

