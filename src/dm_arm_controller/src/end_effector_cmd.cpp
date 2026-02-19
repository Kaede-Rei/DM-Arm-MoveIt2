#include "dm_arm_controller/end_effector_cmd.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

static void shutdown_thread(std::thread& spin_thread);

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

namespace dm_arm {

/**
 * @brief EndEffectorCmd 构造函数：初始化 MoveGroupInterface 并打印规划帧信息
 * @param node 共享指针，指向 ROS 2 节点
 * @param group_name 机械臂控制组的名称
 */
EndEffectorCmd::EndEffectorCmd(rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : _arm_(node, group_name) {
    _node_ = node;

    RCLCPP_INFO(node->get_logger(), "Planning Frame - %s 已创建", _arm_.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End Effector Link - %s 已创建", _arm_.getEndEffectorLink().c_str());
}

/**
 * @brief 设置目标关节值
 * @param joint_values 目标关节值的向量
 * @return 设置是否成功
 */
bool EndEffectorCmd::set_joints(const std::vector<double>& joint_values) {
    return _arm_.setJointValueTarget(joint_values);
}

/**
 * @brief 设置末端执行器的目标位姿、位置或姿态
 * @param target 目标位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion)
 * @return 设置是否成功
 */
bool EndEffectorCmd::set_end(const TargetVariant& target) {
    bool success = false;

    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&target)) {
        success = _arm_.setPoseTarget(*pose);
        RCLCPP_INFO(_node_->get_logger(), "设置目标位姿是否成功：%s", success ? "是" : "否");
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&target)) {
        success = _arm_.setPositionTarget(point->x, point->y, point->z);
        RCLCPP_INFO(_node_->get_logger(), "设置目标位置是否成功：%s", success ? "是" : "否");
    }
    else if(auto* quat = std::get_if<geometry_msgs::msg::Quaternion>(&target)) {
        success = _arm_.setOrientationTarget(quat->x, quat->y, quat->z, quat->w);
        RCLCPP_INFO(_node_->get_logger(), "设置目标姿态是否成功：%s", success ? "是" : "否");
    }
    else {
        RCLCPP_ERROR(_node_->get_logger(), "不支持的目标类型：%s", typeid(target).name());
        return false;
    }

    return success;
}

/**
 * @brief 规划并执行运动
 * @return 规划和执行是否成功
 */
bool EndEffectorCmd::plan_and_execute() {
    RCLCPP_INFO(_node_->get_logger(), "正在规划...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);

    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(_node_->get_logger(), "规划失败，错误码：%d", err_code.val);
        return false;
    }

    RCLCPP_INFO(_node_->get_logger(), "规划成功，正在执行...");
    err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(_node_->get_logger(), "执行失败，错误码：%d", err_code.val);
        return false;
    }

    RCLCPP_INFO(_node_->get_logger(), "执行成功");
    return true;
}

/**
 * @brief 将 Roll-Pitch-Yaw 角转换为四元数
 * @param roll 滚转角（绕 X 轴旋转）
 * @param pitch 俯仰角（绕 Y 轴旋转）
 * @param yaw 偏航角（绕 Z 轴旋转）
 * @return 转换后的四元数
 */
geometry_msgs::msg::Quaternion EndEffectorCmd::rpy_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();

    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = quat.x();
    quat_msg.y = quat.y();
    quat_msg.z = quat.z();
    quat_msg.w = quat.w();

    return quat_msg;
}

/**
 * @brief 将 Roll-Pitch-Yaw 角和位置转换为位姿
 * @param roll 滚转角（绕 X 轴旋转）
 * @param pitch 俯仰角（绕 Y 轴旋转）
 * @param yaw 偏航角（绕 Z 轴旋转）
 * @param x X 坐标
 * @param y Y 坐标
 * @param z Z 坐标
 * @return 转换后的位姿
 */
geometry_msgs::msg::Pose EndEffectorCmd::rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = rpy_to_quaternion(roll, pitch, yaw);
    return pose;
}

/**
 * @brief 获取当前关节值
 * @return 当前关节值的向量
 */
std::vector<double> EndEffectorCmd::get_current_joints() const {
    return _arm_.getCurrentJointValues();
}

/**
 * @brief 获取当前末端执行器的位姿
 * @return 当前末端执行器的位姿
 */
geometry_msgs::msg::Pose EndEffectorCmd::get_current_pose() const {
    return _arm_.getCurrentPose().pose;
}

/**
 * @brief 获取当前机械臂的关节名称
 * @return 当前机械臂的关节名称的向量
 */
std::vector<std::string> EndEffectorCmd::get_current_link_names() const {
    return _arm_.getJointNames();
}

} /* namespace dm_arm */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("end_effector_cmd");
    RCLCPP_INFO(node->get_logger(), "节点：end_effector_cmd 已启动");

    // 需要让开线程让节点 spin 来实时更新状态
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 创建 EndEffectorCmd 实例
    dm_arm::EndEffectorCmd eef_cmd(node, "arm");

    // 获取当前关节值并打印
    std::vector<double> current_joints = eef_cmd.get_current_joints();
    std::vector<std::string> joint_names = eef_cmd.get_current_link_names();
    RCLCPP_INFO(node->get_logger(), "当前关节值：");
    for(size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "%s: %f", joint_names[i].data(), current_joints[i]);
    }

    // 设置目标位置
    geometry_msgs::msg::Point target_point;
    target_point.x = 0.4;
    target_point.y = 0.0;
    target_point.z = 0.4;
    bool success = eef_cmd.set_end(target_point);
    if(!success) {
        shutdown_thread(spin_thread);
        return 1;
    }

    // 规划并执行
    success = eef_cmd.plan_and_execute();
    if(!success) {
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
