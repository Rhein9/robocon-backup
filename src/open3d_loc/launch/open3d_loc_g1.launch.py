from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    open3d_loc_share = FindPackageShare('open3d_loc')

    # 声明 use_sim_time 参数
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

    publish_support_tfs_arg = DeclareLaunchArgument(
        'publish_support_tfs',
        default_value='true',
        description='Publish body/imu/base/motion static TFs for the original G1 setup'
    )

    # 配置文件路径
    config_file = PathJoinSubstitution([
        open3d_loc_share,
        'config',
        'loc_param_g1.yaml'
    ])

    map_file = LaunchConfiguration('map_pcd_file')

    # 静态TF发布节点 - camera_init to odom
    static_tf_camera_init2odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom', '--child-frame-id', 'camera_init'
        ]
    )

    # 静态TF发布节点 - imu_link to base_link
    # 修正：父frame是imu_link，子frame是base_link
    static_tf_imulink2baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imulink2baselink',
        condition=IfCondition(LaunchConfiguration('publish_support_tfs')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'imu_link', '--child-frame-id', 'base_link'
        ]
    )

    # FAST_LIO 发布 camera_init -> body，需要补齐 body -> imu_link 才能连通到 base_link
    static_tf_body2imulink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body2imulink',
        condition=IfCondition(LaunchConfiguration('publish_support_tfs')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'body', '--child-frame-id', 'imu_link'
        ]
    )

    # 静态TF发布节点 - base_link to motion_link
    # 修正：base_link是父frame，motion_link是子frame
    static_tf_base_center = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_center_broadcaster',
        condition=IfCondition(LaunchConfiguration('publish_support_tfs')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link', '--child-frame-id', 'motion_link'
        ]
    )

    # 全局定位节点
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                'path_map': map_file,
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.01,
                'voxelsize_fine': 0.2,
                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,
                'loc_frequence': 2.5,
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_pcd_file_arg,
        publish_support_tfs_arg,
        static_tf_camera_init2odom,
        static_tf_body2imulink,
        static_tf_imulink2baselink,
        static_tf_base_center,
        global_localization_node,
    ])
