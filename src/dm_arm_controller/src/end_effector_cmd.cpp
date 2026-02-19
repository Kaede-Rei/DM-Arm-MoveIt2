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
EndEffectorCmd::EndEffectorCmd(rclcpp::Node::SharedPtr node, const std::string& group_name)
    : _node_(std::move(node)),
    _arm_(_node_, group_name),
    _tf_buffer_(std::make_unique<tf2_ros::Buffer>(_node_->get_clock())),
    _tf_listener_(std::make_unique<tf2_ros::TransformListener>(*_tf_buffer_)),
    _base_link_(_arm_.getPlanningFrame()),
    _eef_link_(_arm_.getEndEffectorLink()) {

    RCLCPP_INFO(_node_->get_logger(), "Planning Frame - %s 已创建", _base_link_.c_str());
    RCLCPP_INFO(_node_->get_logger(), "End Effector Link - %s 已创建", _eef_link_.c_str());

    // 声明参数
    _node_->declare_parameter("motion_planning.planning_time", 5.0);
    _node_->declare_parameter("motion_planning.planning_attempts", 10);
    _node_->declare_parameter("motion_planning.max_velocity_scaling_factor", 0.1);
    _node_->declare_parameter("motion_planning.max_acceleration_scaling_factor", 0.1);
    _node_->declare_parameter("motion_planning.planner_id", "RRTConnect");

    _node_->declare_parameter("target_tolerance.position", 0.001);
    _node_->declare_parameter("target_tolerance.orientation", 0.01);
    _node_->declare_parameter("target_tolerance.joint", 0.01);

    _node_->declare_parameter("tf.timeout", 0.1);
    _node_->declare_parameter("tf.cache_duration", 10.0);

    // 设置参数
    _arm_.setPlanningTime(_node_->get_parameter("motion_planning.planning_time").as_double());
    _arm_.setNumPlanningAttempts(_node_->get_parameter("motion_planning.planning_attempts").as_int());
    _arm_.setMaxVelocityScalingFactor(_node_->get_parameter("motion_planning.max_velocity_scaling_factor").as_double());
    _arm_.setMaxAccelerationScalingFactor(_node_->get_parameter("motion_planning.max_acceleration_scaling_factor").as_double());
    _arm_.setPlannerId(_node_->get_parameter("motion_planning.planner_id").as_string());

    // 等待 1 秒
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 检查 TF 变换是否可用
    try {
        if(_tf_buffer_->canTransform(_base_link_, _eef_link_, rclcpp::Time(0))) {
            RCLCPP_INFO(_node_->get_logger(), "TF 变换可用：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        }
        else {
            RCLCPP_WARN(_node_->get_logger(), "TF 变换不可用：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        }
    }
    catch(const tf2::TransformException& e) {
        RCLCPP_ERROR(_node_->get_logger(), "检查 TF 变换时发生异常：%s", e.what());
    }
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
 * @brief 设置末端执行器的目标位姿、位置或姿态（底座坐标系）
 * @param target 目标位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion)
 * @return 设置是否成功
 */
bool EndEffectorCmd::set_target(const TargetVariant& target) {
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
 * @brief 设置末端执行器的目标位姿、位置或姿态（末端坐标系）
 * @param target 目标位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion)
 * @return 设置是否成功
 */
bool EndEffectorCmd::set_target_on_end(const TargetVariant& target) {
    bool success = false;

    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&target)) {
        auto pose_tmp = *pose;
        base_to_end_tf(pose_tmp, pose_tmp);
        success = _arm_.setPoseTarget(pose_tmp);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标位姿是否成功：%s", success ? "是" : "否");
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&target)) {
        auto point_tmp = *point;
        base_to_end_tf(point_tmp, point_tmp);
        success = _arm_.setPositionTarget(point_tmp.x, point_tmp.y, point_tmp.z);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标位置是否成功：%s", success ? "是" : "否");
    }
    else if(auto* quat = std::get_if<geometry_msgs::msg::Quaternion>(&target)) {
        auto quat_tmp = *quat;
        base_to_end_tf(quat_tmp, quat_tmp);
        success = _arm_.setOrientationTarget(quat_tmp.x, quat_tmp.y, quat_tmp.z, quat_tmp.w);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标姿态是否成功：%s", success ? "是" : "否");
    }
    else {
        RCLCPP_ERROR(_node_->get_logger(), "不支持的目标类型：%s", typeid(target).name());
        return false;
    }

    return true;
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
 * @brief 获取当前末端执行器的位姿（底座坐标系）
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

    // ===== 测试 1: 获取当前关节值和关节名称 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 1] 获取当前关节值和关节名称");
    std::vector<double> current_joints = eef_cmd.get_current_joints();
    std::vector<std::string> joint_names = eef_cmd.get_current_link_names();
    RCLCPP_INFO(node->get_logger(), "当前关节值：");
    for(size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "  %s: %f", joint_names[i].c_str(), current_joints[i]);
    }

    // ===== 测试 2: 获取当前末端执行器位姿 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 2] 获取当前末端执行器位姿");
    geometry_msgs::msg::Pose current_pose = eef_cmd.get_current_pose();
    RCLCPP_INFO(node->get_logger(), "当前位姿 - 位置: (%.3f, %.3f, %.3f)",
        current_pose.position.x, current_pose.position.y, current_pose.position.z);
    RCLCPP_INFO(node->get_logger(), "当前位姿 - 姿态: (%.3f, %.3f, %.3f, %.3f)",
        current_pose.orientation.x, current_pose.orientation.y,
        current_pose.orientation.z, current_pose.orientation.w);

    // ===== 测试 3: RPY 转换函数 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 3] RPY 角转换为四元数");
    double roll = 0.0, pitch = 0.0, yaw = 0.785398; // π/4
    geometry_msgs::msg::Quaternion quat = eef_cmd.rpy_to_quaternion(roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), "RPY(%f, %f, %f) -> 四元数: (%.3f, %.3f, %.3f, %.3f)",
        roll, pitch, yaw, quat.x, quat.y, quat.z, quat.w);

    // ===== 测试 4: RPY 转换为位姿 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 4] RPY 角和位置转换为位姿");
    geometry_msgs::msg::Pose target_pose = eef_cmd.rpy_to_pose(roll, pitch, yaw, 0.3, 0.2, 0.5);
    RCLCPP_INFO(node->get_logger(), "目标位姿 - 位置: (%.3f, %.3f, %.3f)",
        target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(node->get_logger(), "目标位姿 - 姿态: (%.3f, %.3f, %.3f, %.3f)",
        target_pose.orientation.x, target_pose.orientation.y,
        target_pose.orientation.z, target_pose.orientation.w);

    // ===== 测试 5: 设置目标位置（Point）=====
    RCLCPP_INFO(node->get_logger(), "\n[测试 5] 设置目标位置（Point）");
    geometry_msgs::msg::Point target_point;
    target_point.x = 0.4;
    target_point.y = 0.0;
    target_point.z = 0.4;
    bool success = eef_cmd.set_target(target_point);
    RCLCPP_INFO(node->get_logger(), "设置目标位置: (%.3f, %.3f, %.3f) - %s",
        target_point.x, target_point.y, target_point.z,
        success ? "成功" : "失败");

    // ===== 测试 6: 规划并执行运动 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 6] 规划并执行运动");
    success = eef_cmd.plan_and_execute();
    if(!success) {
        RCLCPP_ERROR(node->get_logger(), "规划或执行失败");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "运动执行完成");
    }

    // ===== 测试 7: 设置目标位姿（Pose） =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 7] 设置目标位姿（Pose）");
    geometry_msgs::msg::Pose target_pose2 = eef_cmd.rpy_to_pose(0.0, 0.0, 0.0, 0.35, -0.1, 0.45);
    success = eef_cmd.set_target(target_pose2);
    RCLCPP_INFO(node->get_logger(), "设置目标位姿 - %s", success ? "成功" : "失败");

    success = eef_cmd.plan_and_execute();
    if(!success) {
        RCLCPP_WARN(node->get_logger(), "位姿规划或执行失败，继续测试其他功能");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "位姿运动执行完成");
    }

    // ===== 测试 8: 设置目标姿态（Quaternion） =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 8] 设置目标姿态（Quaternion）");
    geometry_msgs::msg::Quaternion target_quat = eef_cmd.rpy_to_quaternion(0.0, 0.0, 1.5708); // π/2
    success = eef_cmd.set_target(target_quat);
    RCLCPP_INFO(node->get_logger(), "设置目标姿态 - %s", success ? "成功" : "失败");

    success = eef_cmd.plan_and_execute();
    if(!success) {
        RCLCPP_WARN(node->get_logger(), "姿态规划或执行失败");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "姿态运动执行完成");
    }

    // ===== 测试 9: 设置关节值 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 9] 设置目标关节值");
    std::vector<double> target_joints = { 0.0, 0.785398, 1.5708, 0.0, 1.5708, 0.0 };
    success = eef_cmd.set_joints(target_joints);
    RCLCPP_INFO(node->get_logger(), "设置关节值 - %s", success ? "成功" : "失败");

    success = eef_cmd.plan_and_execute();
    if(!success) {
        RCLCPP_WARN(node->get_logger(), "关节值规划或执行失败");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "关节值运动执行完成");
    }

    // ===== 测试 10: 坐标系转换 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 10] 坐标系转换 (Base -> End)");
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 0.5;
    test_pose.position.y = 0.1;
    test_pose.position.z = 0.3;
    test_pose.orientation.x = 0.0;
    test_pose.orientation.y = 0.0;
    test_pose.orientation.z = 0.0;
    test_pose.orientation.w = 1.0;

    geometry_msgs::msg::Pose end_pose;
    if(eef_cmd.base_to_end_tf(test_pose, end_pose)) {
        RCLCPP_INFO(node->get_logger(), "Base -> End 转换成功");
        RCLCPP_INFO(node->get_logger(), "  转换前: (%.3f, %.3f, %.3f)",
            test_pose.position.x, test_pose.position.y, test_pose.position.z);
        RCLCPP_INFO(node->get_logger(), "  转换后: (%.3f, %.3f, %.3f)",
            end_pose.position.x, end_pose.position.y, end_pose.position.z);
    }
    else {
        RCLCPP_WARN(node->get_logger(), "Base -> End 转换失败");
    }

    // ===== 测试 11: 反向坐标系转换 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 11] 坐标系转换 (End -> Base)");
    geometry_msgs::msg::Pose base_pose;
    if(eef_cmd.end_to_base_tf(end_pose, base_pose)) {
        RCLCPP_INFO(node->get_logger(), "End -> Base 转换成功");
        RCLCPP_INFO(node->get_logger(), "  转换前: (%.3f, %.3f, %.3f)",
            end_pose.position.x, end_pose.position.y, end_pose.position.z);
        RCLCPP_INFO(node->get_logger(), "  转换后: (%.3f, %.3f, %.3f)",
            base_pose.position.x, base_pose.position.y, base_pose.position.z);
    }
    else {
        RCLCPP_WARN(node->get_logger(), "End -> Base 转换失败");
    }

    // ===== 测试 12: 设置末端坐标系上的目标位置 =====
    RCLCPP_INFO(node->get_logger(), "\n[测试 12] 设置末端坐标系上的目标位置");
    geometry_msgs::msg::Point end_target_point;
    end_target_point.x = 0.1;
    end_target_point.y = 0.0;
    end_target_point.z = 0.2;
    success = eef_cmd.set_target_on_end(end_target_point);
    RCLCPP_INFO(node->get_logger(), "设置末端坐标系目标位置 - %s", success ? "成功" : "失败");

    success = eef_cmd.plan_and_execute();
    if(!success) {
        RCLCPP_WARN(node->get_logger(), "末端坐标系规划或执行失败");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "末端坐标系运动执行完成");
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
