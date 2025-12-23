import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージ共有ディレクトリを取得
    pkg_share = get_package_share_directory('simple_arm_description')

    # URDF (xacro) ファイルのパス
    urdf_file = os.path.join(pkg_share, 'urdf', 'simple_arm.urdf.xacro')

    # xacroファイルを展開してURDF(XML形式)に変換
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # RViz設定ファイルのパス（rviz/ フォルダに配置）
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'simple_arm.rviz')

    return LaunchDescription([
        # joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])