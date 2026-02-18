#include "dm_arm_controller/end_effector_cmd.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void shutdown_thread(std::thread& spin_thread);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("end_effector_cmd");
    RCLCPP_INFO(node->get_logger(), "节点：end_effector_cmd 已启动");

    // 需要让开线程让节点 spin 来实时更新状态
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 创建 MoveGroupInterface 对象，指定控制的机械臂组名称
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    RCLCPP_INFO(node->get_logger(), "Planning Frame - %s 已创建", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End Effector Link - %s 已创建", arm.getEndEffectorLink().c_str());

    // 获取当前关节值并打印
    std::vector<double> current_joints = arm.getCurrentJointValues();
    std::vector<std::string> joint_names = arm.getJointNames();
    RCLCPP_INFO(node->get_logger(), "当前关节值：");
    for(size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "%s: %f", joint_names[i].data(), current_joints[i]);
    }

    // 设置目标关节值
    std::vector<double> target_joints = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    bool success = arm.setJointValueTarget(target_joints);
    RCLCPP_INFO(node->get_logger(), "设置目标关节值是否成功：%s", success ? "是" : "否");
    if(!success) {
        shutdown_thread(spin_thread);
        return 1;
    }

    // 规划
    RCLCPP_INFO(node->get_logger(), "正在规划...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode err_code = arm.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "规划失败，错误码：%d", err_code.val);
        shutdown_thread(spin_thread);
        return 1;
    }

    // 执行
    RCLCPP_INFO(node->get_logger(), "规划成功，正在执行...");
    err_code = arm.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "执行失败，错误码：%d", err_code.val);
        shutdown_thread(spin_thread);
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "执行成功，目标位置已达成");
    shutdown_thread(spin_thread);

    return 0;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 安全地关闭 ROS 2 节点并等待 spin 线程结束
 * @param spin_thread 负责节点 spin 的线程
 */
static void shutdown_thread(std::thread& spin_thread) {
    rclcpp::shutdown();
    if(spin_thread.joinable()) {
        spin_thread.join();
    }
}
