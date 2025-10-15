import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node,PushRosNamespace
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

base_dir=get_package_share_directory("robot_navigation2")
bt_navigator_dir = get_package_share_directory('robot_navigation2')
map_path=os.path.join(base_dir,"maps","slam_gmapping_test_1.yaml")
param_path=os.path.join(base_dir,"param","param_different.yaml")
bt_xml_filename=os.path.join(bt_navigator_dir,'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
rviz_path=os.path.join(base_dir,'rviz','nav2_default_view.rviz')

use_sim_time = LaunchConfiguration('use_sim_time', default='false')
default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename',default=bt_xml_filename)
autostart = LaunchConfiguration('autostart',default="true")
namespace = LaunchConfiguration('namespace',default="")
use_namespace = LaunchConfiguration('use_namespace',default="false")
map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local',default='false')
map_yaml_file = LaunchConfiguration('map',default=map_path)
params_file = LaunchConfiguration('params_file',default=param_path)


lifecycle_nodes_nav=['map_server', 'amcl','controller_server','planner_server','recoveries_server','bt_navigator','waypoint_follower']


remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/cmd_vel', '/chassis/cmd_vel')]

param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'default_bt_xml_filename':default_bt_xml_filename,
        'autostart':autostart,
        'map_subscribe_transient_local':map_subscribe_transient_local}

configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument("map",default_value=map_yaml_file),
        DeclareLaunchArgument("params_file",default_value=params_file),
        DeclareLaunchArgument("use_sim_time",default_value=use_sim_time),
        DeclareLaunchArgument("namespace",default_value=namespace),
        DeclareLaunchArgument("use_namespace",default_value=use_namespace),
        DeclareLaunchArgument("default_bt_xml_filename",default_value=default_bt_xml_filename),
        DeclareLaunchArgument("autostart",default_value=autostart),
        GroupAction([
            PushRosNamespace(condition=IfCondition(use_namespace),namespace=namespace),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_nav}]),

        ]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
