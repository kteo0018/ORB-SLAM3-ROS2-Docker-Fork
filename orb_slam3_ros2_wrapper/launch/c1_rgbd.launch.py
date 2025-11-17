#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

# MODIFIED: import time, os, config/param, timestamp folder
import time, os

CONFIG_FILE = "c1_configs_x500_rgbd"
PARAM_FILE = "c1-params-x500-rgbd"
# Create timestamped directory for atlas saving
timestamp_dir = f"/root/colcon_ws/atlas_{time.strftime('%y%m%d_%H%M%S')}"
os.makedirs(timestamp_dir, exist_ok=True)

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')
#---------------------------------------------

    # LAUNCH ARGS
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace =  LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value="robot",
        description='The namespace of the robot')
#---------------------------------------------

    def all_nodes_launch(context, robot_namespace):
        params_file = LaunchConfiguration('params_file')
        vocabulary_file_path = "/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        config_file_path = f"/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/{CONFIG_FILE}.yaml"
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(orb_wrapper_pkg, 'params', f'{PARAM_FILE}.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        base_frame = ""
        if(robot_namespace.perform(context) == ""):
            base_frame = ""
        else:
            base_frame = robot_namespace.perform(context) + "/"

        param_substitutions = {
            # 'robot_base_frame': base_frame + 'base_footprint',
            # 'odom_frame': base_frame + 'odom'
            }


        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=robot_namespace.perform(context),
            param_rewrites=param_substitutions,
            convert_types=True)
        
        orb_slam3_node = Node(
            package='orb_slam3_ros2_wrapper',
            executable='rgbd',
            output='screen',
            # prefix=["gdbserver localhost:3000"],
            namespace=robot_namespace.perform(context),
            arguments=[vocabulary_file_path, config_file_path],
            parameters=[configured_params],
            # MODIFIED: ADDED THE BELOW THREE LINES
            cwd=timestamp_dir,
            sigterm_timeout=LaunchConfiguration('sigterm_timeout', default='30'),
            sigkill_timeout=LaunchConfiguration('sigkill_timeout', default='60'))
        
        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch, args=[robot_namespace])
#---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        opaque_function
    ])
