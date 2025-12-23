# Gazebo用のlaunchファイル
# 目的：
#   - URDFテンプレート（.xacro）を展開してURDFを生成
#   - Gazeboを起動
#   - ロボットを自動でスポーン
#
# メリット：
#   - ワンコマンドで「Gazebo起動＋ロボット表示」が可能
#   - xacro修正が即反映される（毎回URDFを生成するため）

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory
import xacro  # xacroファイルを展開するためのモジュール

def generate_launch_description():
    # ==============================
    # 1. パッケージ共有ディレクトリの取得
    # ==============================
    # simple_arm_description パッケージの共有ディレクトリを取得
    pkg_share = get_package_share_directory('simple_arm_description')

    # URDFテンプレート（xacroファイル）のパスを指定
    xacro_file = os.path.join(pkg_share, 'urdf', 'simple_arm.urdf.xacro')

    # ==============================
    # 2. xacroを展開してURDFを生成
    # ==============================
    # xacroファイルを展開してXML形式に変換
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toprettyxml()

    # Gazeboに渡すため、一時ファイルとして保存
    urdf_output = '/tmp/simple_arm.urdf'
    with open(urdf_output, 'w') as f:
        f.write(robot_description)

    # ==============================
    # 3. LaunchDescriptionの定義
    # ==============================
    return LaunchDescription([
        # ------------------------------
        # Gazebo本体を起動
        # ------------------------------
        # gazebo_rosパッケージに含まれる標準のgazebo.launch.pyを呼び出す
        # world引数で空ワールドを明示的に指定
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py')),
            launch_arguments={'world': '/usr/share/gazebo-11/worlds/empty.world'}.items(),
        ),

        # ------------------------------
        # 展開済みURDFをGazeboにスポーン
        # ------------------------------
        # spawn_entity.py はURDFやSDFをGazeboに読み込むためのノード
        # -file : 読み込むURDFファイル
        # -entity : Gazebo上でのモデル名
        # -z : 初期高さ（0.0にすることで地面に直接置く）
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_output, '-entity', 'simple_arm', '-z', '0.0'],
            output='screen'  # 実行ログをターミナルに出力
        )
    ])