import os
from launch.frontend.parser import parse_if_substitutions
from launch_ros.actions import Node
from launch import LaunchDescription, descriptions
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from scipy.signal import resample


def generate_launch_description():

    # Package name
    package_name='amcl_launch'

    # Launch configurations

    playback = LaunchConfiguration('playback', default=False)
    map_yaml = os.path.join(get_package_share_directory(package_name),'map','mapa.yaml')


    map_server_params = os.path.join(get_package_share_directory(package_name),'config','map_server.yaml')
    map_server = Node(
        name='map_server',
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            map_server_params,
            {'yaml_filename': map_yaml},
            {'use_sim_time': True},
        ],
        respawn=True,
        respawn_delay=0.5,
    )

    # rebuild odom tf when playing a bag file
    ekf_localization = Node(
            package='ekf_localization',
            executable='ekf_localization.py',
            respawn=True,
            respawn_delay=0.1,
            parameters=[{'use_sim_time': True}],
            output='screen',
    )

    # rebuild odom tf when playing a bag file
    odom_to_tf = Node(
            package='odom_to_tf_ros2',
            executable='odom_to_tf',
            respawn=True,
            respawn_delay=0.1,
            parameters=[{'use_sim_time': True}],
            output='screen',
            remappings=[('odom/perfect', 'odom')]
    )

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(get_package_share_directory('el7009_diff_drive_robot'),'urdf','robot.urdf.xacro')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('el7009_diff_drive_robot'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )
    
    # Static transform publisher 
    static_transform_publisher_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[
            # '--x', '-144.0', '--y', '38', '--z', '0.0',
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', '/map',
            '--child-frame-id', '/industrial-warehouse'
        ]
    )
    
    # Launch them all!
    description =  LaunchDescription([
        # Declare launch arguments

        # Launch the nodes
        # amcl_node,
        ekf_localization,
        map_server,
    ])
    if playback:
        description.add_action(odom_to_tf)
        description.add_action(rsp)
        description.add_action(static_transform_publisher_map_odom)
    return description
