#############################################################################
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node,LoadComposableNodes
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition

def generate_launch_description():

    # Importtant, required arguments
    points_topic = LaunchConfiguration('points_topic', default='/unilidar/cloud')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='unilidar_lidar') 
    imu_topic = LaunchConfiguration('imu_topic', default='/unilidar/imu')
    globalmap_pcd = DeclareLaunchArgument('globalmap_pcd', default_value='/home/rmml05/thr_ws/src/thr_database/maps/10F_Excellence.pcd',description='Path to the global map PCD file')#scans_1002.pcd', description='Path to the global map PCD file')

    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # optional arguments
    use_imu = LaunchConfiguration('use_imu', default='false')
    invert_imu_acc = LaunchConfiguration('invert_imu_acc', default='false')
    invert_imu_gyro = LaunchConfiguration('invert_imu_gyro', default='false')
    use_global_localization = LaunchConfiguration('use_global_localization', default='true')
    enable_robot_odometry_prediction = LaunchConfiguration('enable_robot_odometry_prediction', default='false')
    robot_odom_frame_id = LaunchConfiguration('robot_odom_frame_id', default='odom')
    plot_estimation_errors = LaunchConfiguration('plot_estimation_errors', default='false')
    send_tf_transforms = LaunchConfiguration('send_tf_transforms', default='false')

    # include hdl_global_localization launch file
    hdl_global_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hdl_localization'),
            'launch/hdl_global_localization.launch.py')),
        condition=IfCondition(use_global_localization),
    )

    hdl_global_localization_path = get_package_share_directory('hdl_global_localization')
    general_config = os.path.join(hdl_global_localization_path, 'config', 'general_config.yaml')
    bbs_config = os.path.join(hdl_global_localization_path, 'config', 'bbs_config.yaml')
    fpfh_config = os.path.join(hdl_global_localization_path, 'config', 'fpfh_config.yaml')
    ransac_config = os.path.join(hdl_global_localization_path, 'config', 'ransac_config.yaml')
    teaser_config = os.path.join(hdl_global_localization_path, 'config', 'teaser_config.yaml')

    # general_config = LaunchConfiguration('general_config')
    # declare_general_config_cmd = DeclareLaunchArgument(
    #     'general_config',
    #     default_value=os.path.join(hdl_global_localization_path, 'config', 'general_config.yaml'),
    #     description='')

    # optional tf arguments
    lidar_tf = Node(
        name='lidar_tf2',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0',
                   '0', '1', 'odom', 'base_link']
    )

    hdl_global_localization = Node(
        package='hdl_global_localization',
        executable='hdl_global_localization_node',
        name='hdl_global_localization',
        output='screen',
        parameters=[general_config,bbs_config,fpfh_config,ransac_config,teaser_config]
    )
    
    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_global_localization',
                plugin='hdl_global_localization::GlobalLocalizationNode',
                name='GlobalLocalizationNode',
                parameters=[general_config,bbs_config,fpfh_config,ransac_config,teaser_config]),
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServerNodelet',
                name='GlobalmapServerNodelet',
                parameters=[
                    {'globalmap_pcd': LaunchConfiguration('globalmap_pcd')},
                    {'convert_utm_to_local': True},
                    {'downsample_resolution': 0.2}]),
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalizationNodelet',
                name='HdlLocalizationNodelet',
                remappings=[('/velodyne_points', points_topic), ('/gpsimu_driver/imu_data', imu_topic)],
                parameters=[
                    {'odom_child_frame_id': odom_child_frame_id},
                    # imu settings
                    {'use_imu': use_imu},
                    {'invert_acc': invert_imu_acc},
                    {'invert_gyro': invert_imu_gyro},
                    {'cool_time_duration': 0.5},
                    # robot odometry-based prediction
                    {'enable_robot_odometry_prediction': enable_robot_odometry_prediction},
                    {'robot_odom_frame_id': robot_odom_frame_id},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {'reg_method': 'NDT_OMP'},
                    {'ndt_neighbor_search_method': 'DIRECT7'},
                    {'ndt_neighbor_search_radius': 2.0},
                    {'ndt_resolution': 0.5},
                    {'downsample_resolution': 0.2},
                    {'specify_init_pose': True},
                    {'init_pos_x': 0.0},
                    {'init_pos_y': 0.0},
                    {'init_pos_z': 0.0},
                    {'init_ori_w': 1.0},
                    {'init_ori_x': 0.0},
                    {'init_ori_y': 0.0},
                    {'init_ori_z': 0.0},
                    {'use_global_localization': use_global_localization},
                    {'send_tf_transforms': send_tf_transforms}])
        ],
        output='screen',   
    )

    # container = Node(
    #     name='container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     output="both",
    # )
    #         # ComposableNode(
    #         #     package='hdl_global_localization',
    #         #     plugin='hdl_global_localization::GlobalLocalizationNode',
    #         #     name='GlobalLocalizationNode',
    #         #     parameters=[general_config,bbs_config,fpfh_config,ransac_config,teaser_config]),
    # Load_Nodes = LoadComposableNodes(
    #     target_container='container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='hdl_localization',
    #             plugin='hdl_localization::GlobalmapServerNodelet',
    #             name='GlobalmapServerNodelet',
    #             parameters=[
    #                 {'globalmap_pcd': LaunchConfiguration('globalmap_pcd')},
    #                 {'convert_utm_to_local': True},
    #                 {'downsample_resolution': 0.2}]),
    #         ComposableNode(
    #             package='hdl_localization',
    #             plugin='hdl_localization::HdlLocalizationNodelet',
    #             name='HdlLocalizationNodelet',
    #             remappings=[('/velodyne_points', points_topic), ('/gpsimu_driver/imu_data', imu_topic)],
    #             parameters=[
    #                 {'odom_child_frame_id': odom_child_frame_id},
    #                 # imu settings
    #                 {'use_imu': use_imu},
    #                 {'invert_acc': invert_imu_acc},
    #                 {'invert_gyro': invert_imu_gyro},
    #                 {'cool_time_duration': 0.5},
    #                 # robot odometry-based prediction
    #                 {'enable_robot_odometry_prediction': enable_robot_odometry_prediction},
    #                 {'robot_odom_frame_id': robot_odom_frame_id},
    #                 # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
    #                 {'reg_method': 'NDT_OMP'},
    #                 {'ndt_neighbor_search_method': 'DIRECT7'},
    #                 {'ndt_neighbor_search_radius': 2.0},
    #                 {'ndt_resolution': 0.5},
    #                 {'downsample_resolution': 0.2},
    #                 {'specify_init_pose': True},
    #                 {'init_pos_x': 0.0},
    #                 {'init_pos_y': 0.0},
    #                 {'init_pos_z': 0.0},
    #                 {'init_ori_w': 1.0},
    #                 {'init_ori_x': 0.0},
    #                 {'init_ori_y': 0.0},
    #                 {'init_ori_z': 0.0},
    #                 {'use_global_localization': use_global_localization},
    #                 {'send_tf_transforms': send_tf_transforms}]),
    #     ], 
    # )

    return LaunchDescription([globalmap_pcd,container])    
