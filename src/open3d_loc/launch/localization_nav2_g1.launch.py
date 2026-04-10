from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox_driver = LaunchConfiguration('start_livox_driver')
    map_pcd_file = LaunchConfiguration('map_pcd_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    use_goal_pose_bridge = LaunchConfiguration('use_goal_pose_bridge')
    log_level = LaunchConfiguration('log_level')
    start_rviz = LaunchConfiguration('start_rviz')
    nav2_start_delay_s = LaunchConfiguration('nav2_start_delay_s')
    fastlio_config_path = LaunchConfiguration('fastlio_config_path')
    fastlio_config_file = LaunchConfiguration('fastlio_config_file')
    publish_support_tfs = LaunchConfiguration('publish_support_tfs')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'localization_3d_g1.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_livox_driver': start_livox_driver,
            'map_pcd_file': map_pcd_file,
            'start_rviz': start_rviz,
            'fastlio_config_path': fastlio_config_path,
            'fastlio_config_file': fastlio_config_file,
            'publish_support_tfs': publish_support_tfs,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'nav2_g1.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'map_yaml_file': map_yaml_file,
            'use_respawn': use_respawn,
            'use_goal_pose_bridge': use_goal_pose_bridge,
            'log_level': log_level,
        }.items(),
    )

    delayed_nav2_launch = TimerAction(
        period=nav2_start_delay_s,
        actions=[nav2_launch],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.',
        ),
        DeclareLaunchArgument(
            'start_livox_driver',
            default_value='true',
            description='Start livox_ros_driver2 together with localization.',
        ),
        DeclareLaunchArgument(
            'map_pcd_file',
            default_value='/home/futureflow/ws_loc/src/data/test.pcd',
            description='Full path to the 3D PCD map used by localization and nav2 occupancy projection.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack.',
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PathJoinSubstitution([
                open3d_loc_share,
                'maps',
                'test_map.yaml',
            ]),
            description='Full path to the 2D YAML map used by nav2 map_server.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                open3d_loc_share,
                'config',
                'nav2_params_g1.yaml',
            ]),
            description='Full path to the nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='true',
            description='Respawn nav2 nodes when they crash.',
        ),
        DeclareLaunchArgument(
            'use_goal_pose_bridge',
            default_value='true',
            description='Bridge RViz /goal_pose to the nav2 navigate_to_pose action.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='Log level for nav2 nodes.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz together with localization and nav2.',
        ),
        DeclareLaunchArgument(
            'nav2_start_delay_s',
            default_value='8.0',
            description='Delay before starting nav2 so localization and TF can settle.',
        ),
        DeclareLaunchArgument(
            'fastlio_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'config',
            ]),
            description='Directory that contains the FAST_LIO configuration file.',
        ),
        DeclareLaunchArgument(
            'fastlio_config_file',
            default_value='mid360.yaml',
            description='FAST_LIO configuration file name.',
        ),
        DeclareLaunchArgument(
            'publish_support_tfs',
            default_value='true',
            description='Publish open3d_loc support TFs for the original G1 setup.',
        ),
        localization_launch,
        delayed_nav2_launch,
    ])
