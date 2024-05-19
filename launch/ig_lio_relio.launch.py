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
    slam_type = LaunchConfiguration('slam_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path').perform(context)
    ig_lio_reloc_dir = get_package_share_directory('lio_relocalization')
    robot_type = LaunchConfiguration('robot_type').perform(context)
    reloc_param = slam_type +"_reloc.yaml"
    config_path += "/" + robot_type +"_velodyne.yaml"

    reloc_param_path = os.path.join(
        ig_lio_reloc_dir,
        'params',
        reloc_param
    )
    lio_relocalization_node = Node(
        package='lio_relocalization',
        executable='lio_relocalization_node',
        name='lio_relocalization_node',
        output='screen',
        parameters=[reloc_param_path, {"map/map_name": LaunchConfiguration('map_name')},{'map/map_location': LaunchConfiguration('map_location')}],  # Pass the parameter file path directly
    )
    lio_tf_fusion_node =  Node(
        package='lio_relocalization',
        executable='lio_tf_fusion_node',
        name='lio_tf_fusion_node',
        output='screen',
        parameters=[reloc_param_path],  # Pass the parameter file path directly
    )

    # ground_removal_node = Node(
    #     package="pointcloud_handler",
    #     executable="filter_pointcloud",
    #     name="ground_removal_node",
    #     respawn=True,
    #     output="screen",
    #     # arguments=['--ros-args', '--log-level', 'WARN'],
    #     parameters=[
    #         {"target_topic": "utlidar/cloud"},
    #         {"target_frame": "utlidar_lidar"},
    #         {"robot_frame": "trunk"},
    #         {"min_height": -0.3},
    #         {"max_height": 2.0},
    #         {"threshold": 0.1}
    #     ]
    # )
    
    print(config_path)
    # static_map_to_odom_node =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='world_to_map',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'lio_odom']
    # )

    ig_lio_node = Node(
        package='ig_lio',
        executable='ig_lio_node',
        name='ig_lio_node',
        output='screen',
        parameters=[config_path]  # Pass the parameter file path directly
    )
    return [
        lio_relocalization_node,
        lio_tf_fusion_node,
        ig_lio_node,
    ]

def generate_launch_description():
    package_path = get_package_share_directory('ig_lio')
    default_config_path = os.path.join(package_path, 'config')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='sh',
        description='Name of the map to be used.')
    slam_type_arg = DeclareLaunchArgument(
        'slam_type',
        default_value='fast_lio',
        description='Slam type to be used.')
    map_location_arg = DeclareLaunchArgument(
        'map_location',
        default_value='',
        description='Map location')
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='go2',
        description='robot_type'
    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_config_path_cmd,
        map_name_arg,
        slam_type_arg,
        map_location_arg,
        robot_type_arg,
        OpaqueFunction(function=launch_setup)
    ])
