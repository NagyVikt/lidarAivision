from launch import LaunchDescription, actions, events
from launch_ros.actions import Node
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


    
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ESP32'],
            output='screen',
            name='micro_ros_agent_usb0',
        ),

        # Node(
        #     package='hoverboard_bridge',
        #     executable='bridge_node',
        #     output='screen'
        # ),

        # Node(
        #     package='bno055_buffer_node',
        #     executable='bno055_buffer_node',
        #     name='bno055_buffer_node',
        #     output='screen'
        # ),


        # Node(
        #     package='odometry_calibration',
        #     executable='odometry_calibration_node',
        #     name='odometry_calibration_node',
        #     output='screen'
        # ),


        # Node(
        #     package='imu_time_corrector_cpp',
        #     executable='time_corrector_node',
        #     output='screen'
        # ),

        # Node(
        #     package='imu_to_euler_converter',
        #     executable='imu_to_euler_node',
        #     output='screen'
        # ),

           # Start imu_filter_madgwick node
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter_madgwick_node',
        #     output='screen',
        #     remappings=[
        #         ('imu/data_raw', '/bno055/imu_raw2'),
        #         ('/imu/mag', '/bno055/mag')

        #     ],
        #     parameters=[{'use_mag': True, 'publish_tf':True, 'fixed_frame': 'bno055_frame', 'imu_frame': 'bno055', 'gain': 0.1,'orientation_stddev': 0.1, 'world_frame': 'enu','sample_freq': 50.0}]
        # ),


            # Wheel Odometry Publisher Node
        # Node(
        #     package='odometry_bridge',
        #     executable='wheel_odometry_publisher.py',
        #     output='screen'
        # ),

        # Node(
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments=['0', '0', '0', '0', '0', '0', '1', 'bno055_frame', 'bno055'],
        #      output='screen'
        #  ),

        # Node(
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments=['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'bno055_frame'],
        #      output='screen'
        #  ),



          # Delayed reset of ESP32 devices
        actions.TimerAction(
            period=1.0,  # Wait for 5 seconds (adjust as needed)
            actions=[
                actions.OpaqueFunction(function=lambda context: reset_esp32("/dev/ESP32")),
                #actions.OpaqueFunction(function=lambda context: reset_esp32("/dev/ttyUSB1")),
            ],
        )
    ])