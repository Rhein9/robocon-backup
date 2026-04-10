from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fast_lio_share = FindPackageShare('fast_lio')
    open3d_loc_share = FindPackageShare('open3d_loc')
    livox_driver_share = FindPackageShare('livox_ros_driver2')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    map_pcd_file_arg = DeclareLaunchArgument(
        'map_pcd_file',
        default_value='/home/futureflow/ws_loc/src/data/test.pcd',
        description='Full path to the PCD map used by global localization'
    )

    start_livox_driver_arg = DeclareLaunchArgument(
        'start_livox_driver',
        default_value='true',
        description='Start livox_ros_driver2 together with localization. Set false when replaying rosbag.'
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz together with localization.'
    )

    fastlio_config_path_arg = DeclareLaunchArgument(
        'fastlio_config_path',
        default_value=PathJoinSubstitution([fast_lio_share, 'config']),
        description='Directory that contains the FAST_LIO configuration file.'
    )

    fastlio_config_file_arg = DeclareLaunchArgument(
        'fastlio_config_file',
        default_value='mid360.yaml',
        description='FAST_LIO configuration file name.'
    )

    publish_support_tfs_arg = DeclareLaunchArgument(
        'publish_support_tfs',
        default_value='true',
        description='Publish open3d_loc support TFs for the original G1 setup.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fast_lio_share,
                'launch',
                'mapping.launch.py'
            ])
        ]),
        launch_arguments={
            'config_path': LaunchConfiguration('fastlio_config_path'),
            'config_file': LaunchConfiguration('fastlio_config_file'),
            'rviz': 'false',
            'use_sim_time': use_sim_time,
        }.items()
    )

    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share,
                'launch',
                'open3d_loc_g1.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_pcd_file': LaunchConfiguration('map_pcd_file'),
            'publish_support_tfs': LaunchConfiguration('publish_support_tfs'),
        }.items()
    )

    livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                livox_driver_share,
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_livox_driver'))
    )

    rviz_config_path = PathJoinSubstitution([
        open3d_loc_share,
        'rviz_cfg',
        'fastlio.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        arguments=['-d', rviz_config_path],
        output='screen',
        prefix='nice',
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_pcd_file_arg,
        start_livox_driver_arg,
        start_rviz_arg,
        fastlio_config_path_arg,
        fastlio_config_file_arg,
        publish_support_tfs_arg,
        livox_driver_launch,
        fast_lio_launch,
        open3d_loc_launch,
        rviz_node
    ])
