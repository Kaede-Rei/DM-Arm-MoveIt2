#include "dm_arm_controller/eef_controller.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

namespace dm_arm {

/**
 * @brief TwoFingerGripper 构造函数
 * @param node 共享指针，指向 ROS 2 节点
 * @param eef_name 末端执行器的名称
 */
TwoFingerGripper::TwoFingerGripper(rclcpp::Node::SharedPtr node, const std::string& eef_name)
    : EndEffector(std::move(node), eef_name),

    _gripper_(this->node(), eef_name) {
    RCLCPP_INFO(this->node()->get_logger(), "末端执行器控制器 [%s] 已创建，规划组：%s", get_eef_name().c_str(), _gripper_.getName().c_str());
}

/**
 * @brief 立即停止末端执行器的动作，通常用于紧急停止或取消当前的运动
 */
void TwoFingerGripper::stop() {
    _gripper_.stop();
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 已停止", get_eef_name().c_str());
}

/**
 * @brief 获取末端执行器的规划组名称
 * @return 规划组名称字符串引用
 */
const std::string& TwoFingerGripper::get_group_name() const {
    return get_eef_name();
}

/**
 * @brief 获取末端执行器的 MoveGroupInterface 对象
 * @return MoveGroupInterface 对象的引用
 */
moveit::planning_interface::MoveGroupInterface& TwoFingerGripper::get_move_group() {
    return _gripper_;
}

/**
 * @brief 打开末端执行器
 * @return 打开结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TwoFingerGripper::open() {
    return execute_preset_pose("open");
}

/**
 * @brief 关闭末端执行器
 * @return 关闭结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TwoFingerGripper::close() {
    return execute_preset_pose("close");
}

/**
 * @brief 执行末端执行器的预设位姿
 * @param pose_name 预设位姿的名称字符串
 * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TwoFingerGripper::execute_preset_pose(const std::string& pose_name) {
    RCLCPP_INFO(node()->get_logger(), "末端执行器控制器 [%s] 执行预设位姿 [%s]", get_eef_name().c_str(), pose_name.c_str());
    _gripper_.setNamedTarget(pose_name);
    _gripper_.move();

    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置末端执行器单个关节的值
 * @param joint_name 关节名称字符串
 * @param value 关节的角度值（单位：弧度）
 * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
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

/**
 * @brief 设置末端执行器所有关节的值
 * @param joint_values 按顺序角度值的向量，单位：弧度
 * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
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

/**
 * @brief 规划末端执行器的运动轨迹
 * @param plan 规划结果的输出参数，包含规划生成的轨迹和相关信息
 * @return 规划结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
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

/**
 * @brief 执行末端执行器的运动轨迹
 * @param plan 需要执行的运动轨迹，包含规划生成的轨迹和相关信息
 * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
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

/**
 * @brief 规划并执行末端执行器设置好的目标状态
 * @return 规划和执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
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

/**
 * @brief 获取末端执行器当前的关节值
 * @return 包含所有关节当前值的向量，顺序与规划组中定义的关节顺序一致
 */
std::vector<double> TwoFingerGripper::get_current_joints() const {
    return _gripper_.getCurrentJointValues();
}

/**
 * @brief 获取末端执行器当前的关节名称列表
 * @return 包含所有关节名称的向量，顺序与规划组中定义的关节顺序一致
 */
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


