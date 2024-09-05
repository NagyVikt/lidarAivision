
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir

import serial

def reset_esp32(port):
    ser = serial.Serial(port, 115200)
    ser.setDTR(False)  # Drop DTR
    ser.flushInput()
    ser.setDTR(True)  # Toggle DTR to reset the ESP32
    ser.close()

def on_exit_event(context):
    print("Node exited. Restarting the ESP32 devices.")
    reset_esp32("/dev/ESP32")


def generate_launch_description():
     # Set the logging level to DEBUG for all nodes launched in this session
    os.environ['RCUTILS_LOGGING_SEVERITY'] = 'DEBUG'


    # ==============================================================================
    # =========================    DEFAULT VARIABLES   =============================
    # ==============================================================================
    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    # Construct the full path to the JSON configuration file
    json_file_path = os.path.join(pkg_share, 'config', 'realsense_config.json')

    default_model_path = os.path.join(pkg_share, 'models/r2d2.urdf')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ==============================================================================
    # ===========================    RVIZ VARIABLES   ==============================
    # ==============================================================================

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_3d.rviz')
    #default_rviz_config_path = os.path.join(pkg_share, 'rviz/slamwithrealsenselidar.rviz')

    # ==============================================================================
    # ===========================    NAV2 VARIABLES   ==============================
    # ==============================================================================

 
    static_map_path = os.path.join(pkg_share, 'maps', 'skynet.yaml')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')

    # ==============================================================================
    # ===========================    LIDAR VARIABLES   =============================
    # ==============================================================================
    #channel_type =  LaunchConfiguration('channel_type', default='serial')
    #inverted = LaunchConfiguration('inverted', default='false')
    #angle_compensate = LaunchConfiguration('angle_compensate', default='true')


    # ======================    RIGHT LIDAR RP2 VARIABLES   ========================
    channel_type_right =  LaunchConfiguration('channel_type', default='serial')
    inverted_right = LaunchConfiguration('inverted', default='false')
    angle_compensate_right = LaunchConfiguration('angle_compensate', default='true')
    serial_portright = LaunchConfiguration('serial_port', default='/dev/rightlidar')
    serial_baudrate_right = LaunchConfiguration('serial_baudrate', default='256000')
    frame_idright = LaunchConfiguration('frame_id', default='lidarright')
    scan_mode_right = LaunchConfiguration('scan_mode', default='Boost')
    topic_nameright = LaunchConfiguration('topic_name', default ='scan_right')



    # =======================    LEFT LIDAR RP1 VARIABLES   ========================
    channel_type_left =  LaunchConfiguration('channel_type', default='serial')
    inverted_left = LaunchConfiguration('inverted', default='false')
    angle_compensate_left = LaunchConfiguration('angle_compensate', default='true')
    serial_portleft = LaunchConfiguration('serial_port', default='/dev/leftlidar')
    serial_baudrate_left = LaunchConfiguration('serial_baudrate', default='256000')
    frame_idleft = LaunchConfiguration('frame_id', default='lidarleft')
    scan_mode_left = LaunchConfiguration('scan_mode', default='Boost')
    topic_nameleft = LaunchConfiguration('topic_name', default ='scan_left')


    #TF
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    # ==============================================================================
    # ========================    DEFAULT DECLARERATIONS    ==========================
    # ==============================================================================

    declare_namespace_cmd = DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace')


    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
      name='gui',
      default_value='True',
      description='Flag to enable joint_state_publisher_gui')


    declare_model_path_cmd = DeclareLaunchArgument(
      name='model', 
      default_value=default_model_path, 
      description='Absolute path to robot urdf file')
    

    declare_scanner_cmd = DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        )


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')


    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
      name='use_robot_state_pub',
      default_value='True',
      description='Whether to start the robot state publisher')

    # ==============================================================================
    # ======================    NAVIGATION2 DECLARERATIONS    ======================
    # ==============================================================================


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


    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')


    # ==============================================================================
    # ==========================    LIDAR DECLARATIONS   ===========================
    # ==============================================================================


    # declare_channel_typeN =  DeclareLaunchArgument(
    #         'channel_type',
    #         default_value=channel_type,
    #         description='Specifying channel type of lidar')

    # declare_invertedN =  DeclareLaunchArgument(
    #         'inverted',
    #         default_value=inverted,
    #         description='Specifying whether or not to invert scan data')

    # declare_angle_compensateN =  DeclareLaunchArgument(
    #         'angle_compensate',
    #         default_value=angle_compensate,
    #         description='Specifying whether or not to enable angle_compensate of scan data')

    # ==============================================================================
    # =======================    LIDAR DECLARATIONS RIGHT   ========================
    # ==============================================================================


    declare_channel_type_right =  DeclareLaunchArgument(
            'channel_type_right',
            default_value=channel_type_right,
            description='Specifying channel type of lidar')
    declare_inverted_right =  DeclareLaunchArgument(
            'inverted_right',
            default_value=inverted_right,
            description='Specifying whether or not to invert scan data')
    declare_angle_compensate_right =  DeclareLaunchArgument(
            'angle_compensate_right',
            default_value=angle_compensate_right,
            description='Specifying whether or not to enable angle_compensate of scan data')

    declare_serial_port_right = DeclareLaunchArgument(
        'serial_port_right',
        default_value=serial_portright,
        description='Specifying usb port to connected right lidar')


    declare_serial_baudrate_right = DeclareLaunchArgument(
            'serial_baudrate_right',
            default_value=serial_baudrate_right,
            description='Specifying usb port baudrate to connected lidar')

    declare_frame_id_right = DeclareLaunchArgument(
            'frame_id_right',
            default_value=frame_idright,
            description='Specifying frame_id of lidar')

    declare_scan_mode_right = DeclareLaunchArgument(
            'scan_mode_right',
            default_value=scan_mode_right,
            description='Specifying scan mode of lidar')    


    declare_topic_name_right = DeclareLaunchArgument(
        'topic_name_right',
        default_value='scan_right',
        description='Specifying topic name of right lidar scan data')


    # ==============================================================================
    # =======================    LIDAR DECLARATIONS LEFT   =========================
    # ==============================================================================

    declare_channel_type_left =  DeclareLaunchArgument(
            'channel_type_left',
            default_value=channel_type_left,
            description='Specifying channel type of lidar')

    declare_inverted_left =  DeclareLaunchArgument(
            'inverted_left',
            default_value=inverted_left,
            description='Specifying whether or not to invert scan data')

    declare_angle_compensate_left=  DeclareLaunchArgument(
            'angle_compensate_left',
            default_value=angle_compensate_left,
            description='Specifying whether or not to enable angle_compensate of scan data')

    declare_serial_baudrate_left = DeclareLaunchArgument(
            'serial_baudrate_left',
            default_value=serial_baudrate_left,
            description='Specifying usb port baudrate to connected lidar')


    declare_frame_id_left = DeclareLaunchArgument(
            'frame_id_left',
            default_value=frame_idleft,
            description='Specifying frame_id of lidar')

    declare_scan_mode_left = DeclareLaunchArgument(
            'scan_mode_left',
            default_value=scan_mode_left,
            description='Specifying scan mode of lidar')

    declare_serial_port_left = DeclareLaunchArgument(
        'serial_port_left',
        default_value=serial_portleft,
        description='Specifying usb port to connected left lidar')

    declare_topic_name_left = DeclareLaunchArgument(
        'topic_name_left',
        default_value='scan_left',
        description='Specifying topic name of left lidar scan data')



    # ==============================================================================
    # ==========================    RVIZ DECLARATIONS   ============================
    # ==============================================================================


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=default_rviz_config_path,
      description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')



    # ==============================================================================
    # ================================    ACTIONS    ===============================
    # ==============================================================================


    # --------------------------------- RVIZ ---------------------------------------

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])    


    # --------------------------------- VISUAL ODOMETRY ----------------------------

    start_visual_odometry = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='both',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 18.0}],
            )


    start_visual_odometry_left = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry_right',
                output='both',
                parameters=[{
                    'laser_scan_topic' : '/scan_left_filtered',
                    'odom_topic' : '/odom',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_footprint',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 5.0}],
            )

    # --------------------------------- LIDAR RIGHT ---------------------------------

    start_rplidar_ros_right = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_right',
            parameters=[{'channel_type':channel_type_right,
                         'serial_port': serial_portright,
                         'serial_baudrate': serial_baudrate_right,
                         'frame_id': frame_idright,
                         'inverted': inverted_right,
                         'angle_compensate': angle_compensate_right,
                         'topic_name': topic_nameright,
                         'scan_mode':scan_mode_right}],
            output='screen')




    start_lidarright_filter =  Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain_right",

            parameters=[
                {"scan_topic": "scan_right"},
                PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "angular_filter_right.yaml"
                ])],
        )


    # --------------------------------- LIDAR LEFT---------------------------------

    start_rplidar_ros_left = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_left',
            parameters=[{'channel_type':channel_type_left,
                         'serial_port': serial_portleft,
                         'serial_baudrate': serial_baudrate_left,
                         'frame_id': frame_idleft,
                         'inverted': inverted_left,
                         'angle_compensate': angle_compensate_left,
                         'topic_name': topic_nameleft,
                         'scan_mode':scan_mode_left}],
            output='screen')



    start_lidarleft_filter =  Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain_left",

            parameters=[
                {"scan_topic": "scan_left"},
                PathJoinSubstitution([
                    get_package_share_directory("laser_filters"),
                    "examples", "angular_filter_left.yaml"
                ])],
        )


    laser_scan_merge_node =  Node(
        package = "laser_scan_merger_cam",
        executable= 'ros2_laser_scan_merger',
        name="ros2_laser_scan_merger",
        output = "screen"

    )

    pointcloud_to_laserscan_merge_node =  Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
                       ],
            parameters = [{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.1,
                'min_height': 0.0,
                'max_height': 0.2,
                'angle_min': -3.1416,  # -π
                'angle_max': 3.1416,   # π
                'angle_increment': 0.0087,  # π/360.0
                'scan_time': 0.33,
                'range_min': 0.35,
                'range_max': 12.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )



    # --------------------------------- Kamera Node ---------------------------------

    camera1_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',
        namespace='camera',
        parameters=[ {'serial_no': '213322072999',
        'base_frame_id': 'base_link',
        'enable_accel' : False,
        'initial_reset': False,
        'enable_gyro': False,
        'enable_color': True,
        'enable_depth': True,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_sync': True,
        'publish_odom_tf': False,
        'publish_tf': True,
        'unite_imu_method':0,
        'use_sim_time': False,
        'pointcloud.allow_no_texture_points': False,
        'pointcloud.enable': True,
        'pointcloud.filter_magnitude': 1,
        'pointcloud.frames_queue_size': 0,
        'pointcloud.ordered_pc': False,
        'pointcloud.pointcloud_qos': 'SYSTEM_DEFAULT',
        'pointcloud.stream_filter': 0,              
        'pointcloud.stream_format_filter': 0,
        'pointcloud.stream_index_filter': 0,
        'json_file_path': json_file_path }]
    )


 # Define the pointcloud_processor node
    pointcloud_processor_node = Node(
        package='pcl_processor',  # Update with your actual package name
        executable='pcl_processor',  # Update with your actual executable name
        name='pointcloud_processor',
        output='screen'
    )








    # --------------------------------- JOINT STATE ---------------------------------

    start_joint_state_publisher_cmd = Node(

      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher')

    # --------------------------------- ROBOT STATE ---------------------------------

    start_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      namespace=namespace,
      parameters=[{'robot_description': Command(['xacro ', model])}],
      remappings=remappings,
      arguments=[default_model_path])
    
    start_static_transform_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        name='static_tf_pub_base_footprint_to_laser_frame'
    )



    # --------------------------------- NAV2 ----------------------------------------

    # start_ros2_navigation_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    #     launch_arguments = {'namespace': namespace,
    #                         'use_namespace': use_namespace,
    #                         'slam': slam,
    #                         'map': map_yaml_file,
    #                         'use_sim_time': use_sim_time,
    #                         'params_file': params_file,
    #                         'default_bt_xml_filename': default_bt_xml_filename,
    #                         'autostart': autostart}.items())



    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart
        }.items()
    )



    # ------------------------------- MICRO-ROS -------------------------------------

    start_micro_ros = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ESP32'],
            output='both',
            name='micro_ros_agent_usb0',
        )

    # --------------------------------- HOVERBOARD-BRIDGE ---------------------------

    start_hoverboard_bridge = LifecycleNode(
            package='hoverboard_bridge',
            name='hoverboard_bridge',
            namespace='',  # Set to an empty string if no specific namespace is needed
            executable='bridge_node',
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
    # Throtle messages

    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        arguments=['messages', 'odom_before', '1.0', 'odom'],
        name='odom_throttle',
        output='screen'
    )


    costmap_clearer_node = Node(
        package='costmap_clearer',
        executable='costmap_clearer_node.py',
        name='costmap_clearer_node',
        output='screen',
    )







    start_transform =  Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["-0.15", "0", "0.425", "0", "0", "0", "base_link", "base_footprint"]
        )

    lifecycle_node =  LifecycleNode(package='demo_lifecycle', executable='lifecycle_talker',
                name='lc_talker', namespace='', output='screen')


    demo_lifecycle_listener =  Node(package='demo_lifecycle', executable='listener', output='screen')
    demo_service_client =  Node(package='demo_lifecycle', executable='service_client', output='screen')


    # --------------------------------- ESP32 --------------------------------------

    ResetESP32 = actions.TimerAction(
            period=1.0,  # Wait for 5 seconds (adjust as needed)
            actions=[
                actions.OpaqueFunction(function=lambda context: reset_esp32("/dev/ESP32")),
                #actions.OpaqueFunction(function=lambda context: reset_esp32("/dev/ttyUSB1")),
            ],
        )



    # ==============================================================================
    # ===============================    LAUNCHER   ================================
    # ==============================================================================

    ld = LaunchDescription()

    # ---------------------------------    DEFAULT  -------------------------------

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_model_path_cmd)

    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # -----------------------------------    NAV2   --------------------------------

    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_namespace_cmd)   
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_scanner_cmd)

    # -------------------------------   LIDARS    ----------------------------------

    # ld.add_action(declare_channel_type)
    # ld.add_action(declare_inverted)
    # ld.add_action(declare_angle_compensate)

    # -------------------------------    LEFT LIDAR - RP1  -------------------------

    ld.add_action(declare_channel_type_left)
    ld.add_action(declare_inverted_left)
    ld.add_action(declare_angle_compensate_left)

    ld.add_action(declare_serial_baudrate_left)

    ld.add_action(declare_serial_port_left)
    ld.add_action(declare_frame_id_left)
    ld.add_action(declare_scan_mode_left)
    ld.add_action(declare_topic_name_left)

    # --------------------------------   RIGHT LIDAR -RP2   ------------------------

    ld.add_action(declare_channel_type_right)
    ld.add_action(declare_inverted_right)
    ld.add_action(declare_angle_compensate_right)

    ld.add_action(declare_serial_baudrate_right)

    ld.add_action(declare_serial_port_right)
    ld.add_action(declare_frame_id_right)
    ld.add_action(declare_scan_mode_right)
    ld.add_action(declare_topic_name_right)


    # ---------------------------------------   RVIZ   -----------------------------

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd) 

    # ==============================================================================
    # ===============================    STARTER   =================================
    # ==============================================================================

    # ************************    START RPLIDAR LEFT  **********************************

    ld.add_action(start_rplidar_ros_left)
    ld.add_action(start_lidarleft_filter)

    #ld.add_action(start_visual_odometry_left)  


    # ************************    START RPLIDAR2 RIGHT   **********************************


    ld.add_action(start_rplidar_ros_right)
    ld.add_action(start_lidarright_filter)

    ld.add_action(laser_scan_merge_node)

    ld.add_action(pointcloud_to_laserscan_merge_node)


    ld.add_action(start_visual_odometry) 



    # ************************   CAMERA NODE  1  **********************************

    ld.add_action(camera1_node)

    ld.add_action(pointcloud_processor_node)


    # ************************   CAMERA NODE  2  **********************************


    # *********************    START ROBOT + URDF  *********************************

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_transform)

    # ***************************    HOVERBOARD  **********************************
    #EZ LETESITI A MICROCONTROLLER KAPCSOLATOT
    ld.add_action(start_micro_ros)

    #EZT MEGKELL IRNI
    #FUNKCIO: KAPCSOLAT A HOVERBOARD ES A ROS KOZOTT
    ld.add_action(start_rqt_robot_steering_node_cmd)
    ld.add_action(start_hoverboard_bridge)

    ld.add_action(lifecycle_node)
    ld.add_action(demo_service_client)

    ld.add_action(ResetESP32)
    #ld.add_action(throttle_node)






    # ***************************    START RVIZ   **********************************

    ld.add_action(start_rviz_cmd)


    # ***********************    START NAV2 LAUNCHER FILE   ************************

    #ld.add_action(start_transform)

    ld.add_action(start_ros2_navigation_cmd)
    #ld.add_action(costmap_clearer_node)


    return ld