#include "dm_arm_controller/eef_controller.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

namespace dm_arm {

TwoFingerGripper::TwoFingerGripper(rclcpp::Node::SharedPtr node, const std::string& eef_name)
    : EndEffector(std::move(node), eef_name),

    _gripper_(this->node(), eef_name) {
    RCLCPP_INFO(this->node()->get_logger(), "末端执行器控制器 [%s] 已创建，规划组：%s", get_eef_name().c_str(), _gripper_.getName().c_str());
}

void TwoFingerGripper::stop() {
    _gripper_.stop();
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 已停止", get_eef_name().c_str());
}

const std::string& TwoFingerGripper::get_group_name() const {
    return get_eef_name();
}

moveit::planning_interface::MoveGroupInterface& TwoFingerGripper::get_move_group() {
    return _gripper_;
}

ErrorCode TwoFingerGripper::open() {
    return execute_preset_pose("open");
}

ErrorCode TwoFingerGripper::close() {
    return execute_preset_pose("close");
}

ErrorCode TwoFingerGripper::execute_preset_pose(const std::string& pose_name) {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 执行预设位姿 [%s]", get_eef_name().c_str(), pose_name.c_str());
    _gripper_.setNamedTarget(pose_name);
    _gripper_.move();

    return ErrorCode::SUCCESS;
}

ErrorCode TwoFingerGripper::set_joint_value(const std::string& joint_name, double value) {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 设置关节 [%s] 的值为 %f", get_eef_name().c_str(), joint_name.c_str(), value);

    bool success = _gripper_.setJointValueTarget(joint_name, value);
    if(!success) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 设置关节值 %.4f 失败，可能超出关节限制", get_eef_name().c_str(), value);
        return ErrorCode::TARGET_OUT_OF_BOUNDS;
    }

    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 设置关节 [%s] 的值为 %f 成功", get_eef_name().c_str(), joint_name.c_str(), value);
    return ErrorCode::SUCCESS;
}

ErrorCode TwoFingerGripper::set_joint_values(const std::vector<double>& joint_values) {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 设置关节值", get_eef_name().c_str());

    const auto& joint_names = _gripper_.getJointNames();
    if(joint_values.size() != joint_names.size()) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 设置关节值失败，提供的关节值数量 %zu 与规划组关节数量 %zu 不匹配", get_eef_name().c_str(), joint_values.size(), joint_names.size());
        return ErrorCode::INVALID_TARGET_TYPE;
    }

    bool success = _gripper_.setJointValueTarget(joint_values);
    if(!success) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 设置关节值失败，可能超出关节限制", get_eef_name().c_str());
        return ErrorCode::TARGET_OUT_OF_BOUNDS;
    }

    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 设置关节值成功", get_eef_name().c_str());
    return ErrorCode::SUCCESS;
}

ErrorCode TwoFingerGripper::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 进行运动规划", get_eef_name().c_str());

    moveit::core::MoveItErrorCode err_code = _gripper_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 运动规划失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 运动规划成功，规划时间: %.4f 秒", get_eef_name().c_str(), plan.planning_time_);
    return ErrorCode::SUCCESS;
}

ErrorCode TwoFingerGripper::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 执行运动轨迹", get_eef_name().c_str());

    moveit::core::MoveItErrorCode err_code = _gripper_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 执行运动轨迹失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 执行运动轨迹成功", get_eef_name().c_str());
    return ErrorCode::SUCCESS;
}

ErrorCode TwoFingerGripper::plan_and_execute() {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 正在规划...", get_eef_name().c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::core::MoveItErrorCode err_code = _gripper_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 规划失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 规划成功，正在执行...", get_eef_name().c_str());
    err_code = _gripper_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 执行失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 执行成功", get_eef_name().c_str());
    return ErrorCode::SUCCESS;
}

std::vector<double> TwoFingerGripper::get_current_joints() const {
    return _gripper_.getCurrentJointValues();
}

std::vector<std::string> TwoFingerGripper::get_current_link_names() const {
    return _gripper_.getLinkNames();
}

// TODO: 实现力反馈相关接口
std::vector<std::string> TwoFingerGripper::get_force_names() const {
    return {};
}
ErrorCode TwoFingerGripper::get_force(const std::string& force_name, double& force_value) const {
    RCLCPP_WARN(node()->get_logger(), "末端执行器控制器 [%s] 暂时不支持获取力反馈，力反馈名称：%s，力反馈数值：%.3f", get_eef_name().c_str(), force_name.c_str(), force_value);
    return ErrorCode::SUCCESS;
}

}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


