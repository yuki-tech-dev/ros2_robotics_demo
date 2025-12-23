from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

#Launchファイル起動で、laser_publisher と rviz2 が同時に起動し、保存したレイアウトで自動表示される。
def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_demo')
    rviz_config = os.path.join(pkg_share, 'config', 'laser_sensor_demo.rviz')

    return LaunchDescription([
        Node(
            package='sensor_demo',
            executable='laser_publisher',
            name='laser_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])