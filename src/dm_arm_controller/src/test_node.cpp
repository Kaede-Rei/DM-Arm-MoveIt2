#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_controller/eef_controller.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

static void shutdown_thread(std::thread& spin_thread);

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("end_effector_cmd");
    RCLCPP_INFO(node->get_logger(), "节点：end_effector_cmd 已启动");

    // 需要让开线程让节点 spin 来实时更新状态
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 创建 ArmController 实例
    dm_arm::ArmController eef_cmd(node, "arm");
    eef_cmd.home();

    // ===== 测试 1: 用 TOTG 异步规划直线 =====
    RCLCPP_INFO(node->get_logger(), "测试 1: 用 TOTG 规划直线");
    geometry_msgs::msg::Pose start_pose = eef_cmd.get_current_pose();
    RCLCPP_INFO(node->get_logger(), "当前末端执行器位姿：位置(%.3f, %.3f, %.3f)，姿态(%.3f, %.3f, %.3f, %.3f)",
        start_pose.position.x, start_pose.position.y, start_pose.position.z,
        start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    geometry_msgs::msg::Pose end_pose = start_pose;
    end_pose.position.x -= 0.2;
    end_pose.position.z += 0.1;
    auto result = eef_cmd.set_line(start_pose, end_pose);
    if(result.error_code == dm_arm::ErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "规划成功，开始执行轨迹");
        eef_cmd.async_execute(result.trajectory);
        while(eef_cmd.is_planning_or_executing()) {
            rclcpp::sleep_for(std::chrono::milliseconds(2000));
            RCLCPP_INFO(node->get_logger(), "正在执行轨迹...");
        }
    }
    else {
        RCLCPP_WARN(node->get_logger(), "规划失败，错误信息：%s", result.message.c_str());
    }

    // ===== 测试 2: 用 TOTG 异步规划曲线 =====
    eef_cmd.home();
    RCLCPP_INFO(node->get_logger(), "测试 2: 用 TOTG 规划曲线");
    geometry_msgs::msg::Pose via_pose = start_pose;
    start_pose.position.x += 0.2;
    start_pose.position.z += 0.1;
    via_pose.position.y += 0.4;
    via_pose.position.z += 0.1;
    result = eef_cmd.set_bezier_curve(start_pose, via_pose, end_pose, 30, 0.01, 0.0, dm_arm::TimeParamMethod::TOTG, 0.1, 0.1);
    if(result.error_code == dm_arm::ErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "曲线规划成功，开始执行轨迹");
        eef_cmd.async_execute(result.trajectory);

        // ===== 测试 3: 异步规划执行期间打印状态 =====
        RCLCPP_INFO(node->get_logger(), "测试 3: 异步规划执行期间打印状态");
        while(eef_cmd.is_planning_or_executing()) {
            RCLCPP_INFO(node->get_logger(), "当前末端执行器位姿：位置(%.3f, %.3f, %.3f)，姿态(%.3f, %.3f, %.3f, %.3f)",
                eef_cmd.get_current_pose().position.x, eef_cmd.get_current_pose().position.y, eef_cmd.get_current_pose().position.z,
                eef_cmd.get_current_pose().orientation.x, eef_cmd.get_current_pose().orientation.y, eef_cmd.get_current_pose().orientation.z, eef_cmd.get_current_pose().orientation.w);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }
    else {
        RCLCPP_WARN(node->get_logger(), "曲线规划失败，错误信息：%s", result.message.c_str());
    }

    // ===== 测试完成 =====
    RCLCPP_INFO(node->get_logger(), "所有测试完成");

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

