from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',            # ← 自分のパッケージ名
            executable='talker',             # ← CMakeLists.txtでadd_executableした実行ファイル名
            name='talker',                   # ノード名
            output='screen'
        ),
        Node(
            package='cpp_pubsub',
            executable='listener',           # ← 同じく実行ファイル名
            name='listener',
            output='screen'
        )
    ])