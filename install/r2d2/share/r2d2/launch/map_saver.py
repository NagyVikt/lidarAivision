#!/usr/bin/env python3

# This is the launch file used for ROS Foxy and older. Launch this file to save a map created in RViz.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # .................. Configurable Arguments .....................

    use_sim_time = True
    map_saver_params_file = 'map_saver.yaml'
    # ...............................................................

    pkg_share = FindPackageShare(package='r2d2').find('r2d2')
    map_save_config = os.path.join(pkg_share, 'params', map_saver_params_file)


    return LaunchDescription([

        DeclareLaunchArgument("map_saver_params_file", default_value=map_save_config, description="Map Saver Configuration File"),

        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            emulate_tty=True,  
            parameters=[LaunchConfiguration('map_saver_params_file')]                  
        ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  
            parameters=[
                {'autostart': True},
                {'node_names': ['map_saver']}]

        ),
  
    ])