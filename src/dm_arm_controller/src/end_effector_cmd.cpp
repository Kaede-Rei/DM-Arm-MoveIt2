#include "dm_arm_controller/end_effector_cmd.hpp"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

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

    _node_->declare_parameter("decartes.vel_scale", 0.1);
    _node_->declare_parameter("decartes.acc_scale", 0.1);
    _node_->declare_parameter("decartes.eef_step", 0.01);
    _node_->declare_parameter("decartes.jump_threshold", 0.0);
    _node_->declare_parameter("decartes.min_success_rate", 0.8);

    // 获取参数
    _vel_scale_ = _node_->get_parameter("decartes.vel_scale").as_double();
    _acc_scale_ = _node_->get_parameter("decartes.acc_scale").as_double();
    _eef_step_ = _node_->get_parameter("decartes.eef_step").as_double();
    _jump_threshold_ = _node_->get_parameter("decartes.jump_threshold").as_double();
    _min_success_rate_ = _node_->get_parameter("decartes.min_success_rate").as_double();

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
 * @brief 将末端执行器复位
 */
void EndEffectorCmd::home() {
    RCLCPP_INFO(_node_->get_logger(), "将末端执行器复位到 home 位姿");
    _arm_.setNamedTarget("home");
    _arm_.move();
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
    else if(auto* pose_stamped = std::get_if<geometry_msgs::msg::PoseStamped>(&target)) {
        success = _arm_.setPoseTarget(pose_stamped->pose);
        RCLCPP_INFO(_node_->get_logger(), "设置目标位姿（带时间戳）是否成功：%s", success ? "是" : "否");
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
        end_to_base_tf(pose_tmp, pose_tmp);
        success = _arm_.setPoseTarget(pose_tmp);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标位姿是否成功：%s", success ? "是" : "否");
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&target)) {
        auto point_tmp = *point;
        end_to_base_tf(point_tmp, point_tmp);
        success = _arm_.setPositionTarget(point_tmp.x, point_tmp.y, point_tmp.z);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标位置是否成功：%s", success ? "是" : "否");
    }
    else if(auto* quat = std::get_if<geometry_msgs::msg::Quaternion>(&target)) {
        auto quat_tmp = *quat;
        end_to_base_tf(quat_tmp, quat_tmp);
        success = _arm_.setOrientationTarget(quat_tmp.x, quat_tmp.y, quat_tmp.z, quat_tmp.w);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标姿态是否成功：%s", success ? "是" : "否");
    }
    else if(auto* pose_stamped = std::get_if<geometry_msgs::msg::PoseStamped>(&target)) {
        auto pose_stamped_tmp = *pose_stamped;
        end_to_base_tf(pose_stamped_tmp, pose_stamped_tmp);
        success = _arm_.setPoseTarget(pose_stamped_tmp.pose);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标位姿（带时间戳）是否成功：%s", success ? "是" : "否");
    }
    else {
        RCLCPP_ERROR(_node_->get_logger(), "不支持的目标类型：%s", typeid(target).name());
        return false;
    }

    return success;
}

/**
 * @brief 清除所有目标位姿、位置和姿态
 */
void EndEffectorCmd::clear_target() {
    _arm_.clearPoseTargets();
    RCLCPP_INFO(_node_->get_logger(), "已清除所有目标位姿");
}

/**
 * @brief 伸缩末端执行器
 * @param length 伸缩长度（正值表示伸长，负值表示缩短）
 * @return 伸缩是否成功
 */
bool EndEffectorCmd::telescopic_end(double length) {
    geometry_msgs::msg::Pose point;
    point.position.x = 0.0;
    point.position.y = 0.0;
    point.position.z = length;
    point.orientation = rpy_to_quaternion(0.0, 0.0, 0.0);

    return set_target_on_end(point);
}

/**
 * @brief 旋转末端执行器
 * @param angle 旋转角度（单位：弧度，正值表示逆时针旋转，负值表示顺时针旋转）
 * @return 旋转是否成功
 */
bool EndEffectorCmd::rotate_end(double angle) {
    geometry_msgs::msg::Pose pose;
    pose = rpy_to_pose(0.0, 0.0, angle, 0.0, 0.0, 0.0);

    return set_target_on_end(pose);
}

/**
 * @brief 规划运动
 * @return 规划是否成功
 */
bool EndEffectorCmd::plan() {
    RCLCPP_INFO(_node_->get_logger(), "正在规划...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(_node_->get_logger(), "规划失败，错误码：%d", err_code.val);
        return false;
    }

    RCLCPP_INFO(_node_->get_logger(), "规划成功");
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
 * @brief 停止当前运动
 */
void EndEffectorCmd::stop() {
    _arm_.stop();
    RCLCPP_INFO(_node_->get_logger(), "已停止当前运动");
}

/**
 * @brief 对给定的轨迹进行时间参数化
 * @param trajectory 需要进行时间参数化的轨迹
 * @param method 时间参数化方法（默认使用 TOTG）
 * @param vel_scale 速度缩放因子（默认值为 0.1）
 * @param acc_scale 加速度缩放因子（默认值为 0.1）
 * @return 时间参数化是否成功
 */
bool EndEffectorCmd::parameterize_time(moveit_msgs::msg::RobotTrajectory& trajectory, TimeParamMethod method, double vel_scale, double acc_scale) {
    // 创建 robot_trajectory::RobotTrajectory 对象
    robot_trajectory::RobotTrajectory rt(_arm_.getRobotModel(), _arm_.getName());
    rt.setRobotTrajectoryMsg(*_arm_.getCurrentState(), trajectory);

    // 根据选择的方法进行时间参数化
    bool time_param_success = false;
    if(method == TimeParamMethod::TOTG) {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        time_param_success = totg.computeTimeStamps(rt, vel_scale, acc_scale);

    }
    else if(method == TimeParamMethod::SPLINE) {
        trajectory_processing::IterativeSplineParameterization isp;
        time_param_success = isp.computeTimeStamps(rt, vel_scale, acc_scale);
    }
    else {
        RCLCPP_ERROR(_node_->get_logger(), "无效的时间参数化方法");
        return false;
    }

    if(!time_param_success) {
        RCLCPP_ERROR(_node_->get_logger(), "时间参数化失败");
        return false;
    }

    // 将时间参数化后的轨迹转换回消息格式
    rt.getRobotTrajectoryMsg(trajectory);
    return true;
}

/**
 * @brief 规划并执行 Descartes 轨迹
 * @param waypoints 笛卡尔路径的路径点列表
 * @param eef_step 末端执行器步长（单位：米，默认值为 0.01）
 * @param jump_threshold 跳跃阈值（单位：弧度，默认值为 0.0，表示禁用跳跃检测）
 * @param time_param_method 时间参数化方法（默认使用 TOTG）
 * @param vel_scale 速度缩放因子（默认值为 0.1）
 * @param acc_scale 加速度缩放因子（默认值为 0.1）
 * @return 规划结果，包括是否成功、生成的轨迹和任何错误信息
 */
DescartesResult EndEffectorCmd::plan_decartes(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step, double jump_threshold, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.success = false;

    if(waypoints.empty()) {
        result.message = "路径点列表为空";
        return result;
    }

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    double success_rate = _arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    result.success_rate = success_rate;

    if(success_rate <= 0.0) {
        result.message = "笛卡尔路径规划失败，无法生成有效的轨迹";
        return result;
    }

    // 时间参数化
    if(!parameterize_time(trajectory, time_param_method, vel_scale, acc_scale)) {
        result.message = "时间参数化失败";
        return result;
    }
    result.trajectory = trajectory;

    if(success_rate < _min_success_rate_) {
        result.success = false;
        std::stringstream ss;
        ss << "笛卡尔路径规划失败，成功率：" << (success_rate * 100.0) << "%";
        result.message = ss.str();
    }
    else {
        result.success = true;
        std::stringstream ss;
        ss << "笛卡尔路径规划成功，成功率：" << (success_rate * 100.0) << "%";
        result.message = ss.str();
    }

    return result;
}

/**
 * @brief 规划并执行一条直线路径，如果传入的参数为位置，则保持当前姿态不变
 * @param start 起点目标（位姿或位置）
 * @param end 终点目标（位姿或位置）
 * @param eef_step 末端执行器步长（单位：米，默认值为 0.01）
 * @param jump_threshold 跳跃阈值（单位：弧度，默认值为 0.0，表示禁用跳跃检测）
 * @param time_param_method 时间参数化方法（默认使用 TOTG）
 * @param vel_scale 速度缩放因子（默认值为 0.1）
 * @param acc_scale 加速度缩放因子（默认值为 0.1）
 * @return 规划结果，包括是否成功、生成的轨迹和任何错误信息
 */
DescartesResult EndEffectorCmd::set_line(const TargetVariant& start, const TargetVariant& end, double eef_step, double jump_threshold, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    // 将起点和终点转换为位姿
    geometry_msgs::msg::Pose start_pose;
    geometry_msgs::msg::Pose end_pose;

    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&start)) {
        start_pose = *pose;
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&start)) {
        start_pose.position = *point;
        start_pose.orientation = get_current_pose().orientation;
    }
    else {
        DescartesResult result;
        result.success = false;
        result.message = "无效的起点类型，必须为 geometry_msgs::msg::Pose 或 geometry_msgs::msg::Point";
        return result;
    }
    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&end)) {
        end_pose = *pose;
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&end)) {
        end_pose.position = *point;
        end_pose.orientation = get_current_pose().orientation;
    }
    else {
        DescartesResult result;
        result.success = false;
        result.message = "无效的终点类型，必须为 geometry_msgs::msg::Pose 或 geometry_msgs::msg::Point";
        return result;
    }

    // 创建路径点列表
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(end_pose);

    // 调用笛卡尔路径规划函数
    return plan_decartes(waypoints, eef_step, jump_threshold, time_param_method, vel_scale, acc_scale);
}

/**
 * @brief 规划并执行一条圆弧路径，如果传入的参数为位置，则保持当前姿态不变
 * @param start 起点目标（位姿或位置）
 * @param via 圆弧路径上的一个中间点（位姿或位置）
 * @param end 终点目标（位姿或位置）
 * @param arc_segments 圆弧路径的分段数（默认值为 30，表示将圆弧分成 30 段进行规划）
 * @param eef_step 末端执行器步长（单位：米，默认值为 0.01）
 * @param jump_threshold 跳跃阈值（单位：弧度，默认值为 0.0，表示禁用跳跃检测）
 * @param time_param_method 时间参数化方法（默认使用 TOTG）
 * @param vel_scale 速度缩放因子（默认值为 0.1）
 * @param acc_scale 加速度缩放因子（默认值为 0.1）
 * @return 规划结果，包括是否成功、生成的轨迹和任何错误信息
 */
DescartesResult EndEffectorCmd::set_circle(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int arc_segments, double eef_step, double jump_threshold, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    // 将起点、途经点和终点转换为位姿
    geometry_msgs::msg::Pose start_pose;
    geometry_msgs::msg::Pose via_pose;
    geometry_msgs::msg::Pose end_pose;

    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&start)) {
        start_pose = *pose;
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&start)) {
        start_pose.position = *point;
        start_pose.orientation = get_current_pose().orientation;
    }
    else {
        DescartesResult result;
        result.success = false;
        result.message = "无效的起点类型，必须为 geometry_msgs::msg::Pose 或 geometry_msgs::msg::Point";
        return result;
    }
    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&via)) {
        via_pose = *pose;
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&via)) {
        via_pose.position = *point;
        via_pose.orientation = get_current_pose().orientation;
    }
    else {
        DescartesResult result;
        result.success = false;
        result.message = "无效的途经点类型，必须为 geometry_msgs::msg::Pose 或 geometry_msgs::msg::Point";
        return result;
    }
    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&end)) {
        end_pose = *pose;
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&end)) {
        end_pose.position = *point;
        end_pose.orientation = get_current_pose().orientation;
    }
    else {
        DescartesResult result;
        result.success = false;
        result.message = "无效的终点类型，必须为 geometry_msgs::msg::Pose 或 geometry_msgs::msg::Point";
        return result;
    }

    // 计算圆弧路径上的分段点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for(int i = 0; i <= arc_segments; ++i) {
        double t = static_cast<double>(i) / arc_segments;

        // 使用二次贝塞尔曲线公式计算圆弧路径上的点
        geometry_msgs::msg::Pose point;
        point.position.x = (1 - t) * (1 - t) * start_pose.position.x + 2 * (1 - t) * t * via_pose.position.x + t * t * end_pose.position.x;
        point.position.y = (1 - t) * (1 - t) * start_pose.position.y + 2 * (1 - t) * t * via_pose.position.y + t * t * end_pose.position.y;
        point.position.z = (1 - t) * (1 - t) * start_pose.position.z + 2 * (1 - t) * t * via_pose.position.z + t * t * end_pose.position.z;

        tf2::Quaternion quat_start, quat_end, quat_interp;
        tf2::fromMsg(start_pose.orientation, quat_start);
        tf2::fromMsg(end_pose.orientation, quat_end);
        quat_interp = quat_start.slerp(quat_end, t);
        quat_interp.normalize();
        point.orientation = tf2::toMsg(quat_interp);

        waypoints.push_back(point);
    }

    return plan_decartes(waypoints, eef_step, jump_threshold, time_param_method, vel_scale, acc_scale);
}

/**
 * @brief 按预设轨迹执行运动
 * @param trajectory 预设的运动轨迹
 * @return 规划和执行是否成功
 */
bool EndEffectorCmd::execute(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    RCLCPP_INFO(_node_->get_logger(), "正在按预设轨迹执行...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
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
    eef_cmd.home();

    // ===== 测试 1: 用 TOTG 规划直线 =====
    RCLCPP_INFO(node->get_logger(), "测试 1: 用 TOTG 规划直线");
    geometry_msgs::msg::Pose start_pose = eef_cmd.get_current_pose();
    RCLCPP_INFO(node->get_logger(), "当前末端执行器位姿：位置(%.3f, %.3f, %.3f)，姿态(%.3f, %.3f, %.3f, %.3f)",
        start_pose.position.x, start_pose.position.y, start_pose.position.z,
        start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);
    geometry_msgs::msg::Pose end_pose = start_pose;
    end_pose.position.x -= 0.2;
    end_pose.position.z += 0.1;
    auto result = eef_cmd.set_line(start_pose, end_pose);
    // if(result.success) {
    //     RCLCPP_INFO(node->get_logger(), "规划成功，开始执行轨迹");
    //     eef_cmd.execute(result.trajectory);
    // }
    // else {
    //     RCLCPP_ERROR(node->get_logger(), "规划失败，错误信息：%s", result.message.c_str());
    // }

    // ===== 测试 2: 用 SPLINE 规划圆弧 =====
    RCLCPP_INFO(node->get_logger(), "测试 2: 用 TOTG 规划圆弧");
    geometry_msgs::msg::Pose via_pose = start_pose;
    start_pose.position.x += 0.2;
    start_pose.position.z += 0.1;
    via_pose.position.y += 0.4;
    via_pose.position.z += 0.1;
    result = eef_cmd.set_circle(start_pose, via_pose, end_pose, 30, 0.01, 0.0, dm_arm::TimeParamMethod::TOTG, 0.05, 0.05);
    if(result.success) {
        RCLCPP_INFO(node->get_logger(), "圆弧规划成功，开始执行轨迹");
        eef_cmd.execute(result.trajectory);
    }
    else {
        RCLCPP_ERROR(node->get_logger(), "圆弧规划失败，错误信息：%s", result.message.c_str());
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
