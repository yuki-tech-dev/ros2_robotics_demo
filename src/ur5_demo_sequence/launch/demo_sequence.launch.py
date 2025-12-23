from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. ur_moveit_config の起動
    ur_moveit_config_path = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )

    ur_moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_config_path),
        launch_arguments={'ur_type': 'ur5'}.items()
    )

    # 2. 自作ノード demo_sequence の起動
    # → robot_description は ur_moveit_config がすでに設定しているので、ここでは指定不要
    demo_node = Node(
        package="ur5_demo_sequence",
        executable="demo_sequence",
        name="ur5_demo_sequence",
        output="screen"
    )

    # 3. 両方をまとめて起動
    return LaunchDescription([ur_moveit_config_launch, demo_node])
