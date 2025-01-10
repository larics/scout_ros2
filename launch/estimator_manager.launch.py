import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=launch.substitutions.EnvironmentVariable('UAV_NAMESPACE')
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description="Enable sim time"
    )

    lora_topic_arg = DeclareLaunchArgument(
        'lora_topic',
        default_value="lora_in",
        description="Name of the lora measurement topic"
    )

    ar_tag_topic_arg = DeclareLaunchArgument(
        'ar_tag_topic',
        default_value="ar_tag_in",
        description="Name of the ar_tag measurement topic"
    )

    mavros_imu_topic_arg = DeclareLaunchArgument(
        'mavros_imu_topic',
        default_value="mavros/imu/data",
        description="Name of the Mavros IMU topic"
    )

    mavros_odom_topic_arg = DeclareLaunchArgument(
        'mavros_odom_topic',
        default_value="mavros/global_position/local",
        description="Name of the Mavros global position topic"
    )

    movella_imu_topic_arg = DeclareLaunchArgument(
        'movella_imu_topic',
        default_value="/imu/data",
        description="Name of the Movella IMU topic"
    )
    
    pressure_topic_arg = DeclareLaunchArgument(
        'pressure_topic',
        default_value="mavros/imu/static_pressure",
        description="Name of the pressure topic"
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value="cmd_vel_in",
        description="Name of the command velocity topic for the UAV"
    )
    
    cmd_att_topic_arg = DeclareLaunchArgument(
        'cmd_att_topic',
        default_value="cmd_att_topic",
        description="Name of the command attitude topic for the UAV"
    )

    uav_cmd_topic_arg = DeclareLaunchArgument(
        'uav_cmd_topic',
        default_value="uav_cmd_topic",
        description="Name of the UAVcommand topic for the UAV"
    )

    mavros_gps_raw_fix_topic_arg = DeclareLaunchArgument(
        'mavros_gps_raw_fix_topic',
        default_value="mavros_gps_raw_fix_topic",
        description="Name of the gps raw fix input"
    )
    mavros_gps_raw_vel_topic_arg = DeclareLaunchArgument(
    'mavros_gps_raw_vel_topic',
    default_value="mavros_gps_raw_vel_topic",
    description="Name of the gps raw vel input"
    )

    mavros_home_topic_arg = DeclareLaunchArgument(
    'mavros_home_topic',
    default_value="mavros_home_topic",
    description="Name of the gps raw vel input"
    )

    mavros_state_topic_arg = DeclareLaunchArgument(
        'mavros_state_topic',
        default_value="mavros/state",
        description="Name of the mavros/state topic"
    )

    carrot_status_arg = DeclareLaunchArgument(
        'carrot_status_topic',
        default_value="carrot/status",
        description="Name of the carrot/status topic"
    )

    height_config_arg = launch.actions.DeclareLaunchArgument(
        'height_config',
        default_value=os.path.join(
                get_package_share_directory('uav_ros_estimators'),
                'config',
                'height_estimator_config.yaml'
            ),
        description='Height estimator configuration'
        )
    
    horizontal_config_arg = launch.actions.DeclareLaunchArgument(
        'horizontal_config',
        default_value=os.path.join(
                get_package_share_directory('uav_ros_estimators'),
                'config',
                'horizontal_estimator_config.yaml'
            ),
        description='Horizontal estimator configuration'
        )

    heading_config_arg = launch.actions.DeclareLaunchArgument(
        'heading_config',
        default_value=os.path.join(
                get_package_share_directory('uav_ros_estimators'),
                'config',
                'heading_estimator_config.yaml'
            ),
        description='Heading estimator configuration'
        )
    
    general_config_arg = launch.actions.DeclareLaunchArgument(
        'general_config',
        default_value=os.path.join(
                get_package_share_directory('uav_ros_estimators'),
                'config',
                'general_estimator_config.yaml'
            ),
        description='General estimator configuration'
        )
    
    
    container = ComposableNodeContainer(
            name='estimator_container',
            package='rclcpp_components',
            executable='component_container_mt',
            # executable='component_container',
            namespace=namespace,
            composable_node_descriptions=[
                ComposableNode(
                    package='uav_ros_estimators',
                    plugin='uav_ros_estimators::UAVEstimatorManager',
                    name='estimator_manager',
                    namespace=namespace,
                    parameters = [
                        launch.substitutions.LaunchConfiguration('height_config'),
                        launch.substitutions.LaunchConfiguration('horizontal_config'),
                        launch.substitutions.LaunchConfiguration('heading_config'),
                        launch.substitutions.LaunchConfiguration('general_config'),
                        {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=[
                        ('lora_in', launch.substitutions.LaunchConfiguration('lora_topic')),
                        ('ar_tag_in', launch.substitutions.LaunchConfiguration('ar_tag_topic')),
                        ('mavros_imu_in', launch.substitutions.LaunchConfiguration('mavros_imu_topic')), 
                        ('movella_imu_in', launch.substitutions.LaunchConfiguration('movella_imu_topic')), 
                        ('pressure_in', launch.substitutions.LaunchConfiguration('pressure_topic')), 
                        ('cmd_vel_in', launch.substitutions.LaunchConfiguration('cmd_vel_topic')), 
                        ('cmd_att_in', launch.substitutions.LaunchConfiguration('cmd_att_topic')),
                        ('uav_cmd_in', launch.substitutions.LaunchConfiguration('uav_cmd_topic')),
                        ('carrot/status', launch.substitutions.LaunchConfiguration('carrot_status_topic')), 
                        ('mavros/state', launch.substitutions.LaunchConfiguration('mavros_state_topic')),
                        ('mavros_odom_in', launch.substitutions.LaunchConfiguration('mavros_odom_topic')),
                        ('mavros_gps_raw_fix_in', launch.substitutions.LaunchConfiguration('mavros_gps_raw_fix_topic')),
                        ('mavros_gps_raw_vel_in', launch.substitutions.LaunchConfiguration('mavros_gps_raw_vel_topic')),
                        ('mavros/global_position/home', launch.substitutions.LaunchConfiguration('mavros_home_topic')),
                    ],
                    # parameters=[{'width': 320, 'height': 240, 'burger_mode': True, 'history': 'keep_last'}],
                    # extra_arguments=[{'use_intra_process_comms': True}]),
                    )
            ],
            # prefix=["debug_ros2launch"],
            output='both',
    )

    return launch.LaunchDescription([
        use_sim_time_arg,
        mavros_imu_topic_arg,
        mavros_odom_topic_arg,
        movella_imu_topic_arg,
        lora_topic_arg,
        ar_tag_topic_arg,
        pressure_topic_arg,
        cmd_vel_topic_arg,
        cmd_att_topic_arg,
        uav_cmd_topic_arg,
        carrot_status_arg,
        heading_config_arg,
        horizontal_config_arg,
        height_config_arg,
        general_config_arg,
        mavros_state_topic_arg,
        mavros_gps_raw_fix_topic_arg,
        mavros_gps_raw_vel_topic_arg,
        mavros_home_topic_arg,
        namespace_arg,
        container])
