from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the log level argument
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='movella',  # Change this to your preferred default namespace
        description='Namespace for the ntrip_client'
    )

    ntrip_config_arg = DeclareLaunchArgument(
        'ntrip_config',
        default_value=os.path.join(
                get_package_share_directory('ntrip'),
                'config',
                'ntrip_config.yaml'
            ),
        description='NTRIP configuration'
        )

    # Create the node configuration
    ntrip_node = Node(
        package='ntrip',
        executable='ntrip',
        name='ntrip_client',
        output='screen',
        parameters=[LaunchConfiguration('ntrip_config'),],
        # Topic Remapping
        remappings=[
            ('/nmea', [LaunchConfiguration('namespace'), '/nmea']),
            ('/rtcm', [LaunchConfiguration('namespace'), '/rtcm']),
        ],
        # Add arguments for log level
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        log_level,  # Include the log level argument
		namespace_arg,
        ntrip_config_arg,
        ntrip_node  # Include the node configuration
    ])
