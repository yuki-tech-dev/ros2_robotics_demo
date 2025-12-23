#from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_move_group_launch

#def generate_launch_description():
#    moveit_config = (
#        MoveItConfigsBuilder("simple_arm", package_name="simple_arm_moveit_config")
#        .robot_description(file_path="urdf/simple_arm.urdf.xacro")
#        .robot_description_semantic(file_path="config/simple_arm.srdf")
#        .planning_pipelines(pipelines=["ompl"])   # ← file_pathは不要
#        .to_moveit_configs()
#   )
#    return generate_move_group_launch(moveit_config)

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch
# from launch import LaunchDescription

# def generate_launch_description():
#     # MoveIt の設定をまとめるビルダーを作成
#     moveit_config = (
#         MoveItConfigsBuilder("simple_arm", package_name="simple_arm_moveit_config")
#         # URDF（ロボットの形状・リンク構造）を読み込む
#         .robot_description(file_path="urdf/simple_arm.urdf.xacro")
#         # SRDF（MoveIt 用のグループ定義や joint 設定）を読み込む
#         .robot_description_semantic(file_path="config/simple_arm.srdf")
#         # 使用するプランニングパイプラインを指定（ここでは OMPL）
#         .planning_pipelines(pipelines=["ompl"])
#         # 軌跡実行に使うコントローラ設定を読み込む（controllers.yaml）
#         .trajectory_execution(file_path="config/controllers.yaml")
#         #.kinematics(file_path="config/kinematics.yaml")


#         # 最終的に MoveItConfigs オブジェクトへ変換
#         .to_moveit_configs()
#     )
#     # MoveGroup ノードを起動する LaunchDescription を生成
#     return generate_move_group_launch(moveit_config)






# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, TimerAction
# from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterValue
# from ament_index_python.packages import get_package_share_directory
# import os

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch

# def generate_launch_description():
#     # ============================
#     # MoveIt の設定
#     # ============================
#     moveit_config = (
#         MoveItConfigsBuilder("simple_arm", package_name="simple_arm_moveit_config")
#         .robot_description(file_path="urdf/simple_arm.urdf.xacro")
#         .robot_description_semantic(file_path="config/simple_arm.srdf")
#         .planning_pipelines(pipelines=["ompl"])
#         .trajectory_execution(file_path="config/controllers.yaml")
#         .to_moveit_configs()
#     )

#     # ============================
#     # Launch引数としてYAMLファイルを渡す
#     # ============================
#     controllers_yaml_default = os.path.join(
#         get_package_share_directory("simple_arm_moveit_config"),
#         "config",
#         "ros2_controllers.yaml"
#     )

#     declare_yaml_arg = DeclareLaunchArgument(
#         "controller_yaml_file",
#         default_value=controllers_yaml_default,
#         description="Path to ros2_controllers.yaml"
#     )

#     controller_yaml = LaunchConfiguration("controller_yaml_file")

#     # ============================
#     # ros2_control_node の起動
#     # ============================
#     ros2_control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         name="controller_manager",
#         parameters=[
#             {
#                 "robot_description": ParameterValue(
#                     Command([
#                         "xacro ",
#                         PathJoinSubstitution([
#                             FindPackageShare("simple_arm_description"),
#                             "urdf",
#                             "simple_arm.urdf.xacro"
#                         ])
#                     ]),
#                     value_type=str
#                 )
#             },
#             controller_yaml  # ← 明示的にLaunchConfigurationで渡す
#         ],
#         output="screen",
#     )

#     # ============================
#     # spawner ノード（遅延起動）
#     # ============================
#     joint_state_spawner = TimerAction(
#         period=3.0,
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#             )
#         ]
#     )

#     arm_controller_spawner = TimerAction(
#         period=4.0,
#         actions=[
#             Node(
#                 package="controller_manager",
#                 executable="spawner",
#                 arguments=["arm_controller", "--controller-manager", "/controller_manager"],
#             )
#         ]
#     )

#     # ============================
#     # MoveGroup ノード
#     # ============================
#     move_group = generate_move_group_launch(moveit_config)

#     # ============================
#     # LaunchDescription にまとめる
#     # ============================
#     return LaunchDescription([
#         declare_yaml_arg,
#         ros2_control_node,
#         joint_state_spawner,
#         arm_controller_spawner,
#         move_group,
#     ])




from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # ============================
    # MoveIt の設定
    # ============================
    moveit_config = (
        MoveItConfigsBuilder("simple_arm", package_name="simple_arm_moveit_config")
        .robot_description(file_path="urdf/simple_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_arm.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/controllers.yaml")
        .to_moveit_configs()
    )

    # ============================
    # ros2_controllers.yaml のファイルパス
    # ============================
    controllers_yaml_file = os.path.join(
        get_package_share_directory("simple_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    # ============================
    # ros2_control_node の起動
    # ============================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        PathJoinSubstitution([
                            FindPackageShare("simple_arm_description"),
                            "urdf",
                            "simple_arm.urdf.xacro"
                        ])
                    ]),
                    value_type=str
                )
            },
            controllers_yaml_file  # ← controller_manager に YAML を渡す
        ],
        output="screen",
    )

    # ============================
    # spawner ノード（遅延起動）
    # ============================
    joint_state_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
                name="spawner_joint_state_broadcaster"
            )
        ]
    )

    arm_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
                name="spawner_arm_controller"
            )
        ]
    )

    # ============================
    # MoveGroup ノード
    # ============================
    move_group = generate_move_group_launch(moveit_config)

    # ============================
    # LaunchDescription にまとめる
    # ============================
    return LaunchDescription([
        ros2_control_node,
        joint_state_spawner,
        arm_controller_spawner,
        move_group,
    ])
