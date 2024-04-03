import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
def launch_setup(context, *args, **kwargs):
    # Define the 'ig_lio' package directory
    ig_lio_dir = get_package_share_directory('ig_lio')
    
    # Define the path to your parameter file
    param_path = os.path.join(ig_lio_dir, 'config', 'hesai.yaml')
    map_name = LaunchConfiguration('map_name').perform(context)
    map_location = LaunchConfiguration('map_location').perform(context)

    ig_lio_node =   Node(
        package='ig_lio',
        executable='ig_lio_node',
        name='ig_lio_node',
        output='screen',
        parameters=[param_path],  # Pass the parameter file path directly
    )
    
    ig_lio_map_node =  Node(
        package='ig_lio',
        executable='ig_lio_map_node',
        name='ig_lio_map_node',
        output='screen',
        parameters=[param_path, {'map/map_name': map_name}, {'map/map_location': map_location}],  # Pass the parameter file path directly
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'lio_odom']
    )

    return [
        ig_lio_node,
        ig_lio_map_node,
        map_to_odom_tf
    ]
def generate_launch_description():

    declare_map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='sh',
        description='Map name'
    )
    declare_map_location_arg = DeclareLaunchArgument(
        'map_location', default_value='',
        description='Map location'
    )

    return LaunchDescription([
        declare_map_name_arg,
        declare_map_location_arg,
        OpaqueFunction(function=launch_setup)
    ])
