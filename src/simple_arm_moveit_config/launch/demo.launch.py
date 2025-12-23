from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("simple_arm", package_name="simple_arm_moveit_config")
        .robot_description(file_path="urdf/simple_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/simple_arm.srdf")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    demo_ld = generate_demo_launch(moveit_config)
    return LaunchDescription([*demo_ld.entities])
