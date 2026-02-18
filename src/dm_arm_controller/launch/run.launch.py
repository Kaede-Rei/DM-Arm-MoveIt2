from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "dm_arm_description", package_name="dm_arm_moveit_config"
    ).to_moveit_configs()

    # 创建节点并传入 moveit_config.to_dict()
    node = Node(
        package="dm_arm_controller",
        executable="end_effector_cmd",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([node])
