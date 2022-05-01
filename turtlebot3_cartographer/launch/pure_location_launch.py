# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from nav2_common.launch import HasNodeParams, RewrittenYaml
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    cartographer_map_dir = LaunchConfiguration('cartographer_map_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'map'))
    # cartographer_map_dir = LaunchConfiguration('cartographer_map_dir',default=os.path.join
    # (turtlebot3_cartographer_prefix,'map'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='pure_localization.lua')
    rplidar_ros2_dir = get_package_share_directory('rplidar_ros2')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer.rviz')
    teb_launch_dir = os.path.join(
        get_package_share_directory('teb_local_planner'), 'params')
    laser_filters_params = LaunchConfiguration('laser_filters_params',default=os.path.join(turtlebot3_cartographer_prefix, 'config', 'laser_filters.yaml'))
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('scan','base_scan'),
                  ('scan_filtered','scan'),
                  ('chassis_imu','imu')]
    autostart = LaunchConfiguration('autostart')
    # lifecycle_nodes = ['map_server']
    lifecycle_nodes = ['map_saver','map_server']
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    param_substitutions = {
        'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'params_file',
        # default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            default_value=os.path.join(teb_launch_dir, 'teb_params_add.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': "/home/njtech/RMUA_WS/src/turtlebot3/turtlebot3_cartographer/config/map.yaml"},
                        {'use_sim_time': use_sim_time}],
            # parameters=[configured_params],
            remappings=remappings),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
        'laser_filters_params',
         default_value=laser_filters_params,
        description='config of laser_filters'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_ros2_dir, 'launch','rplidar_launch.py'))
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['-0.013308', '-0.10685', '-0.0825', '0', '0', '3.14159', 'base_link', 'lidar_link']),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.18', '0', '0.262', '0', '3.141592654', '0', 'base_link', 'lidar_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.18', '0', '0', '0', '0', '0', 'base_link', 'chassis_imu_link']),
        Node(
            package='roborts_base',
            executable='roborts_base_node',
            remappings=remappings),

        Node(
            package='laser_filters',
            executable = 'scan_to_scan_filter_chain',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("turtlebot3_cartographer"),
                    "config", "laser_filters.yaml",])],
            remappings=remappings),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])