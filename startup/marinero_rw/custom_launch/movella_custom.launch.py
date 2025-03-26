from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # Declare the namespace launch argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='movella',  # Change this to your preferred default namespace
        description='Namespace for the xsens_mti_node'
    )

    ld = LaunchDescription()

    # Set environment variables to control logging behavior
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    parameters_file_path = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node.yaml')

    xsens_mti_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        #namespace=LaunchConfiguration('namespace'),  # Use the namespace from LaunchConfiguration
        output='screen',
        parameters=[parameters_file_path, {'use_namespace': True}], 
        arguments=[],
        remappings=[
            ('/filter/euler', [LaunchConfiguration('namespace'), '/filter/euler']),
            ('/filter/free_acceleration', [LaunchConfiguration('namespace'), '/filter/free_acceleration']),
            ('/filter/positionlla', [LaunchConfiguration('namespace'), '/filter/positionlla']),
            ('/filter/quaternion', [LaunchConfiguration('namespace'), '/filter/quaternion']),
            ('/filter/twist', [LaunchConfiguration('namespace'), '/filter/twist']),
            ('/filter/velocity', [LaunchConfiguration('namespace'), '/filter/velocity']),
            ('/gnss', [LaunchConfiguration('namespace'), '/gnss']),
            ('/gnss_pose', [LaunchConfiguration('namespace'), '/gnss_pose']),
            ('/imu/acceleration', [LaunchConfiguration('namespace'), '/imu/acceleration']),
            ('/imu/angular_velocity', [LaunchConfiguration('namespace'), '/imu/angular_velocity']),
            ('/imu/data', [LaunchConfiguration('namespace'), '/imu/data']),
            ('/imu/mag', [LaunchConfiguration('namespace'), '/imu/mag']),
            ('/imu/time_ref', [LaunchConfiguration('namespace'), '/imu/time_ref']),
            ('/imu/utctime', [LaunchConfiguration('namespace'), '/imu/utctime']),
            ('/nmea', [LaunchConfiguration('namespace'), '/nmea']),
            ('/pressure', [LaunchConfiguration('namespace'), '/pressure']),
            ('/rtcm', [LaunchConfiguration('namespace'), '/rtcm']),
            ('/status', [LaunchConfiguration('namespace'), '/status']),
            ('/temperature', [LaunchConfiguration('namespace'), '/temperature']),
            ('/tf', [LaunchConfiguration('namespace'), '/tf']),
        ]
    )

    # Add actions to the launch description
    ld.add_action(namespace_arg)
    ld.add_action(xsens_mti_node)

    return ld