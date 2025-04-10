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
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition

def generate_launch_description():

    hdl_global_localization_path = get_package_share_directory('hdl_global_localization')
    general_config = os.path.join(hdl_global_localization_path, 'config', 'general_config.yaml')
    bbs_config = os.path.join(hdl_global_localization_path, 'config', 'bbs_config.yaml')
    fpfh_config = os.path.join(hdl_global_localization_path, 'config', 'fpfh_config.yaml')
    ransac_config = os.path.join(hdl_global_localization_path, 'config', 'ransac_config.yaml')
    teaser_config = os.path.join(hdl_global_localization_path, 'config', 'teaser_config.yaml')

    hdl_global_localization = Node(
        package='hdl_global_localization',
        executable='hdl_global_localization_node',
        name='hdl_global_localization',
        output='screen',
        parameters=[general_config,bbs_config,fpfh_config,ransac_config,teaser_config]
    )
    
    return LaunchDescription([hdl_global_localization])    