
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    rplidar_ros2_dir = get_package_share_directory('rplidar_ros2')
    teb_launch_dir = os.path.join(
        get_package_share_directory('teb_local_planner'), 'launch')
    teb_param_dir = os.path.join(
        get_package_share_directory('teb_local_planner'), 'params')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    turtlebot3_cartographer_dir = os.path.join(turtlebot3_cartographer_prefix, 'launch')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_debug = LaunchConfiguration('use_debug')


    laser_filters_params = LaunchConfiguration('laser_filters_params',default=os.path.join(bringup_dir, 'params', 'laser_filters.yaml'))
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('scan','base_scan'),
                  ('scan_filtered','scan'),
                  ('chassis_imu','imu')]
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        # 'use_sim_time': test_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_use_debug_cmd = DeclareLaunchArgument(
        'use_debug', default_value='False',
        description='Whether to use gdb')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        # default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        default_value=os.path.join(teb_launch_dir, 'teb_params_add.yaml'),
        # default_value=os.path.join(teb_param_dir, 'teb_params_add.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')
        
    DeclareLaunchArgument(
        'laser_filters_params',
         default_value=laser_filters_params,
        description='config of laser_filters')
    # print("test"+PythonExpression(['not ', slam]))

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),

            PythonLaunchDescriptionSource(os.path.join(turtlebot3_cartographer_prefix, 'launch','nav2_cartographer.launch.py')),

            # PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_cartographer_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file
                              }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_ros2_dir, 'launch','rplidar_launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),
        Node(
            condition=IfCondition(PythonExpression(
                ['not ', use_debug, ' and ', use_composition])),
            package='nav2_bringup',
            executable='composed_bringup',
            output='screen',
            parameters=[configured_params, {'autostart': autostart}],
            remappings=remappings),
        Node(
            condition=IfCondition(PythonExpression(
                [use_debug, ' and ', use_composition])),
            package='nav2_bringup',
            executable='composed_bringup',
            output='screen',
            parameters=[configured_params, {'autostart': autostart}],
            # prefix=[
            #     'stterm -g 200x60 -e gdb -ex run --args'],
            # prefix=[
            #     'screen -d -m gdb -ex run --args'],
            prefix=['valgrind'],
                
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
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.038', '0.107', '-0.10', '0', '0', '3.14159265', 'base_link', 'lidar_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.18', '0', '0', '0', '0', '0', 'base_link', 'chassis_imu_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.21', '0', '0.35', '1.5707', '0', '1.85877', 'base_link', 'camera_link']),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'local_adjust_link']),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0.21', '0.01', '0.35', '0', '0.27925', '3.141592654', 'base_link', 'ncaa_link']),
        Node(
            package='roborts_base',
            executable='roborts_base_node',
            remappings=remappings),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(turtlebot3_cartographer_dir, 'navigation_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['-0.013308', '-0.10685', '-0.0825', '0', '0', '1.5708', 'base_link', 'lidar_link']),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
