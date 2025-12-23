# 必要なモジュールをインポート
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF (xacro) ファイルのパスを取得
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'simple_arm.urdf.xacro'
    )

    # xacroファイルを展開してURDF(XML形式)に変換
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # LaunchDescription に含めるノードを定義
    return LaunchDescription([
        # ジョイント操作用GUIノード
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # URDFを読み込み、TFを配信するノード
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        # rviz2を起動するノード
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
            os.path.dirname(__file__), '..', 'config', 'simple_arm.rviz')]

        )
    ])