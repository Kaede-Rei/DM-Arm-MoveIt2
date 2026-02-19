from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "dm_arm_description", package_name="dm_arm_moveit_config"
    ).to_moveit_configs()

    # 获取参数服务器路径
    config_file = os.path.join(
        get_package_share_directory("dm_arm_controller"), "config", "params.yaml"
    )

    # 创建节点并传入 moveit_config.to_dict()
    node = Node(
        package="dm_arm_controller",
        executable="end_effector_cmd",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            config_file,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([node])
