#include "dm_arm_controller/arm_controller.hpp"

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

namespace dm_arm {

/**
 * @brief ArmController 构造函数：初始化 MoveGroupInterface 并打印规划帧信息
 * @param node 共享指针，指向 ROS 2 节点
 * @param group_name 机械臂控制组的名称
 */
ArmController::ArmController(rclcpp::Node::SharedPtr node, const std::string& group_name)
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
        RCLCPP_WARN(_node_->get_logger(), "检查 TF 变换时发生异常：%s", e.what());
    }
}

/**
 * @brief ArmController 析构函数：等待异步线程结束
 */
ArmController::~ArmController() {
    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }
}

/**
 * @brief 将末端执行器复位
 */
void ArmController::home() {
    RCLCPP_INFO(_node_->get_logger(), "将末端执行器复位到 home 位姿");
    _arm_.setNamedTarget("home");
    _arm_.move();
}

/**
 * @brief 设置目标关节值
 * @param joint_values 目标关节值的向量
 * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::set_joints(const std::vector<double>& joint_values) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }
    bool success = _arm_.setJointValueTarget(joint_values);
    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
}

/**
 * @brief 设置末端执行器的目标位姿、位置或姿态（底座坐标系）
 * @param target 目标位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion)
 * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::set_target(const TargetVariant& target) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

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
        success = _arm_.setPoseTarget(*pose_stamped);
        RCLCPP_INFO(_node_->get_logger(), "设置目标位姿（带时间戳）是否成功：%s", success ? "是" : "否");
    }
    else {
        RCLCPP_WARN(_node_->get_logger(), "不支持的目标类型：%s", typeid(target).name());
        return ErrorCode::INVALID_TARGET_TYPE;
    }

    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
}

/**
 * @brief 设置末端执行器的目标位姿、位置或姿态（末端坐标系）
 * @param target 目标位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion)
 * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::set_target_in_eef_frame(const TargetVariant& target) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

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
        success = _arm_.setPoseTarget(pose_stamped_tmp);
        RCLCPP_INFO(_node_->get_logger(), "设置末端坐标系目标位姿（带时间戳）是否成功：%s", success ? "是" : "否");
    }
    else {
        RCLCPP_WARN(_node_->get_logger(), "不支持的目标类型：%s", typeid(target).name());
        return ErrorCode::INVALID_TARGET_TYPE;
    }

    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
}

/**
 * @brief 清除所有目标位姿、位置和姿态
 */
void ArmController::clear_target() {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法清除目标");
        return;
    }
    _arm_.clearPoseTargets();
    RCLCPP_INFO(_node_->get_logger(), "已清除所有目标位姿");
}

/**
 * @brief 伸缩末端执行器
 * @param length 伸缩长度（正值表示伸长，负值表示缩短）
 * @return 伸缩结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::telescopic_end(double length) {
    geometry_msgs::msg::Pose point;
    point.position.x = 0.0;
    point.position.y = 0.0;
    point.position.z = length;
    point.orientation = rpy_to_quaternion(0.0, 0.0, 0.0);

    return set_target_in_eef_frame(point);
}

/**
 * @brief 旋转末端执行器
 * @param angle 旋转角度（单位：弧度，正值表示逆时针旋转，负值表示顺时针旋转）
 * @return 旋转结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::rotate_end(double angle) {
    geometry_msgs::msg::Pose pose;
    pose = rpy_to_pose(0.0, 0.0, angle, 0.0, 0.0, 0.0);

    return set_target_in_eef_frame(pose);
}

/**
 * @brief 规划运动
 * @param plan 规划结果，包含轨迹等信息
 * @return 规划结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法进行新的规划");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    RCLCPP_INFO(_node_->get_logger(), "正在规划...");

    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(_node_->get_logger(), "规划失败，错误码：%d", err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "规划成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行规划好的运动
 * @param plan 规划结果，包含轨迹等信息
 * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法执行新的规划结果");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    RCLCPP_INFO(_node_->get_logger(), "正在执行规划结果...");

    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(_node_->get_logger(), "执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 规划并执行运动
 * @return 规划和执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::plan_and_execute() {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法进行新的规划和执行");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    RCLCPP_INFO(_node_->get_logger(), "正在规划...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(_node_->get_logger(), "规划失败，错误码：%d", err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "规划成功，正在执行...");
    err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(_node_->get_logger(), "执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 异步规划并执行运动
 * @param callback 可选的回调函数，在规划和执行完成后调用，参数为 ErrorCode 类型，表示规划和执行的结果，返回值为 void
 * @return 是否成功启动异步任务的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::async_plan_and_execute(std::function<void(ErrorCode)> callback) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，请稍后再试");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    _is_planning_or_executing_ = true;
    RCLCPP_INFO(_node_->get_logger(), "正在异步规划...");

    // 启动一个新的线程来执行规划和执行
    _async_thread_ = std::thread([this, callback]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);
        if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(_node_->get_logger(), "异步规划失败，错误码：%d", err_code.val);
            _is_planning_or_executing_ = false;
            if(callback) callback(ErrorCode::PLANNING_FAILED);
        }
        else {
            RCLCPP_INFO(_node_->get_logger(), "异步规划成功，正在执行...");
            err_code = _arm_.execute(plan);
            if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(_node_->get_logger(), "异步执行失败，错误码：%d", err_code.val);
                _is_planning_or_executing_ = false;
                if(callback) callback(ErrorCode::EXECUTION_FAILED);
            }
            else {
                RCLCPP_INFO(_node_->get_logger(), "异步执行成功");
                _is_planning_or_executing_ = false;
                if(callback) callback(ErrorCode::SUCCESS);
            }
        }
        });

    return ErrorCode::SUCCESS;
}

/**
 * @brief 停止当前运动
 */
void ArmController::stop() {
    _arm_.stop();
    RCLCPP_INFO(_node_->get_logger(), "已停止当前运动");
}

/**
 * @brief 对给定的轨迹进行时间参数化
 * @param trajectory 需要进行时间参数化的轨迹
 * @param method 时间参数化方法（默认使用 TOTG）
 * @param vel_scale 速度缩放因子（默认值为 0.1）
 * @param acc_scale 加速度缩放因子（默认值为 0.1）
 * @return 时间参数化是否成功的错误码，成功返回ErrorCode::SUCCESS，失败返回ErrorCode::TIME_PARAM_FAILED
 */
ErrorCode ArmController::parameterize_time(moveit_msgs::msg::RobotTrajectory& trajectory, TimeParamMethod method, double vel_scale, double acc_scale) {
    // 创建 robot_trajectory::RobotTrajectory 对象
    robot_trajectory::RobotTrajectory rt(_arm_.getRobotModel(), _arm_.getName());
    rt.setRobotTrajectoryMsg(*_arm_.getCurrentState(), trajectory);

    // 根据选择的方法进行时间参数化
    bool time_param_success = false;
    if(method == TimeParamMethod::TOTG) {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        time_param_success = totg.computeTimeStamps(rt, vel_scale, acc_scale);

    }
    else if(method == TimeParamMethod::ISP) {
        trajectory_processing::IterativeSplineParameterization isp;
        time_param_success = isp.computeTimeStamps(rt, vel_scale, acc_scale);
    }
    else {
        RCLCPP_WARN(_node_->get_logger(), "无效的时间参数化方法");
        return ErrorCode::TIME_PARAM_FAILED;
    }

    if(!time_param_success) {
        RCLCPP_WARN(_node_->get_logger(), "时间参数化失败");
        return ErrorCode::TIME_PARAM_FAILED;
    }

    // 将时间参数化后的轨迹转换回消息格式
    rt.getRobotTrajectoryMsg(trajectory);
    return ErrorCode::SUCCESS;
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
DescartesResult ArmController::plan_decartes(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step, double jump_threshold, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::SUCCESS;

    if(_is_planning_or_executing_) {
        result.message = "当前已有异步任务正在执行，无法进行新的笛卡尔规划";
        return result;
    }

    if(waypoints.empty()) {
        result.error_code = ErrorCode::EMPTY_WAYPOINTS;
        result.message = "路径点列表为空";
        return result;
    }

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    double success_rate = _arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    result.success_rate = success_rate;

    if(success_rate <= 0.0) {
        result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;
        result.message = "笛卡尔路径规划失败，无法生成有效的轨迹";
        return result;
    }

    // 时间参数化
    if(parameterize_time(trajectory, time_param_method, vel_scale, acc_scale) != ErrorCode::SUCCESS) {
        result.error_code = ErrorCode::TIME_PARAM_FAILED;
        result.message = "时间参数化失败";
        return result;
    }
    result.trajectory = trajectory;

    if(success_rate < _min_success_rate_) {
        result.error_code = ErrorCode::SUCCESS;
        std::stringstream ss;
        ss << "笛卡尔路径规划失败，成功率：" << (success_rate * 100.0) << "%";
        result.message = ss.str();
    }
    else {
        result.error_code = ErrorCode::SUCCESS;
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
DescartesResult ArmController::set_line(const TargetVariant& start, const TargetVariant& end, double eef_step, double jump_threshold, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
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
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
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
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
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
 * @brief 规划并执行一条二次贝塞尔曲线路径，如果传入的参数为位置，则保持当前姿态不变
 * @param start 起点目标（位姿或位置）
 * @param via 曲线路径上的一个中间点（位姿或位置），用于定义曲线的形状
 * @param end 终点目标（位姿或位置）
 * @param curve_segments 曲线路径的分段数（默认值为 30，表示将曲线分成 30 段进行规划）
 * @param eef_step 末端执行器步长（单位：米，默认值为 0.01）
 * @param jump_threshold 跳跃阈值（单位：弧度，默认值为 0.0，表示禁用跳跃检测）
 * @param time_param_method 时间参数化方法（默认使用 TOTG）
 * @param vel_scale 速度缩放因子（默认值为 0.1）
 * @param acc_scale 加速度缩放因子（默认值为 0.1）
 * @return 规划结果，包括是否成功、生成的轨迹和任何错误信息
 */
DescartesResult ArmController::set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments, double eef_step, double jump_threshold, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
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
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
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
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
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
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "无效的终点类型，必须为 geometry_msgs::msg::Pose 或 geometry_msgs::msg::Point";
        return result;
    }

    // 计算曲线路径上的分段点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for(int i = 0; i <= curve_segments; ++i) {
        double t = static_cast<double>(i) / curve_segments;

        // 使用二次贝塞尔曲线公式计算曲线路径上的点
        geometry_msgs::msg::Pose point;
        point.position.x = (1 - t) * (1 - t) * start_pose.position.x + 2 * (1 - t) * t * via_pose.position.x + t * t * end_pose.position.x;
        point.position.y = (1 - t) * (1 - t) * start_pose.position.y + 2 * (1 - t) * t * via_pose.position.y + t * t * end_pose.position.y;
        point.position.z = (1 - t) * (1 - t) * start_pose.position.z + 2 * (1 - t) * t * via_pose.position.z + t * t * end_pose.position.z;

        // 对姿态进行球面线性插值
        tf2::Quaternion quat_start, quat_end;
        tf2::Quaternion quat_interp;
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
 * @return 规划和执行是否成功的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::execute(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，无法执行新轨迹");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    RCLCPP_INFO(_node_->get_logger(), "正在按预设轨迹执行...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(_node_->get_logger(), "执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 异步按预设轨迹执行运动
 * @param trajectory 预设的运动轨迹
 * @param callback 可选的回调函数，在执行完成后调用，参数为 ErrorCode 类型，表示执行结果，返回值为 void
 * @return 是否成功启动异步执行任务的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::async_execute(const moveit_msgs::msg::RobotTrajectory& trajectory, std::function<void(ErrorCode)> callback) {
    if(_is_planning_or_executing_) {
        RCLCPP_WARN(_node_->get_logger(), "当前已有异步任务正在执行，请稍后再试");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    _is_planning_or_executing_ = true;
    RCLCPP_INFO(_node_->get_logger(), "正在异步按预设轨迹执行...");

    // 启动一个新的线程来执行轨迹
    _async_thread_ = std::thread([this, trajectory, callback]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);
        if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(_node_->get_logger(), "异步执行失败，错误码：%d", err_code.val);
            _is_planning_or_executing_ = false;
            if(callback) callback(ErrorCode::EXECUTION_FAILED);
        }
        else {
            RCLCPP_INFO(_node_->get_logger(), "异步执行成功");
            _is_planning_or_executing_ = false;
            if(callback) callback(ErrorCode::SUCCESS);
        }
        });

    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置末端执行器的姿态约束
 * @param target_orientation 目标姿态（四元数）
 * @param tolerance_x 姿态约束在 X 轴上的容忍度（单位：弧度，默认值为 0.1）
 * @param tolerance_y 姿态约束在 Y 轴上的容忍度（单位：弧度，默认值为 0.1）
 * @param tolerance_z 姿态约束在 Z 轴上的容忍度（单位：弧度，默认值为 0.3）
 * @param weight 约束权重（默认值为 1.0，表示硬约束，值越小表示对约束的满足要求越低）
 */
void ArmController::set_orientation_constraint(const geometry_msgs::msg::Quaternion& target_orientation, double tolerance_x, double tolerance_y, double tolerance_z, double weight) {
    // 创建姿态约束对象
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = _eef_link_;
    ocm.header.frame_id = _base_link_;
    ocm.header.stamp = _node_->now();

    // 设置目标姿态和容忍度
    ocm.orientation = target_orientation;
    ocm.absolute_x_axis_tolerance = tolerance_x;
    ocm.absolute_y_axis_tolerance = tolerance_y;
    ocm.absolute_z_axis_tolerance = tolerance_z;
    ocm.weight = weight;

    // 将姿态约束添加到约束列表中
    _constraints_.orientation_constraints.push_back(ocm);
    RCLCPP_INFO(_node_->get_logger(), "已设置末端执行器姿态约束，目标姿态：(%f, %f, %f, %f)，容忍度：(%f, %f, %f)，权重：%f",
        target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w,
        tolerance_x, tolerance_y, tolerance_z, weight);
}

/**
 * @brief 设置末端执行器的位置约束
 * @param target_position 目标位置
 * @param scope_size 位置约束的范围大小（单位：米）
 * @param weight 约束权重（默认值为 1.0，表示硬约束，值越小表示对约束的满足要求越低）
 */
void ArmController::set_position_constraint(const geometry_msgs::msg::Point& target_position, const geometry_msgs::msg::Vector3& scope_size, double weight) {
    // 创建位置约束对象
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.link_name = _eef_link_;
    pcm.header.frame_id = _base_link_;
    pcm.header.stamp = _node_->now();

    // 设置目标位置
    pcm.target_point_offset.x = target_position.x;
    pcm.target_point_offset.y = target_position.y;
    pcm.target_point_offset.z = target_position.z;

    // 定义一个立方体作为位置约束的边界
    shape_msgs::msg::SolidPrimitive bounding_volume;
    bounding_volume.type = shape_msgs::msg::SolidPrimitive::BOX;
    bounding_volume.dimensions.resize(3);
    bounding_volume.dimensions = { scope_size.x, scope_size.y, scope_size.z };
    pcm.constraint_region.primitives.push_back(bounding_volume);
    pcm.constraint_region.primitive_poses.push_back(geometry_msgs::msg::Pose());
    pcm.weight = weight;

    // 将位置约束添加到约束列表中
    _constraints_.position_constraints.push_back(pcm);
    RCLCPP_INFO(_node_->get_logger(), "已设置末端执行器位置约束，目标位置：(%f, %f, %f)，范围大小：(%f, %f, %f)，权重：%f",
        target_position.x, target_position.y, target_position.z,
        scope_size.x, scope_size.y, scope_size.z, weight);
}

/**
 * @brief 设置末端执行器的关节约束
 * @param joint_name 需要约束的关节名称
 * @param target_angle 目标关节角度（单位：弧度）
 * @param upper 关节角度的上容忍度（单位：弧度，默认值为 0.1）
 * @param lower 关节角度的下容忍度（单位：弧度，默认值为 0.1）
 * @param weight 约束权重（默认值为 1.0，表示硬约束，值越小表示对约束的满足要求越低）
 */
void ArmController::set_joint_constraint(const std::string& joint_name, double target_angle, double above, double below, double weight) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = joint_name;
    jc.position = target_angle;
    jc.tolerance_above = above;
    jc.tolerance_below = below;
    jc.weight = weight;

    _constraints_.joint_constraints.push_back(jc);
    RCLCPP_INFO(_node_->get_logger(), "已设置末端执行器关节约束，关节名称：%s，目标角度：%f，上容忍度：%f，下容忍度：%f，权重：%f",
        joint_name.c_str(), target_angle, above, below, weight);
}

/**
 * @brief 应用所有姿态约束
 */
void ArmController::apply_constraints() {
    _arm_.setPathConstraints(_constraints_);
    RCLCPP_INFO(_node_->get_logger(), "已应用所有姿态约束");
}

/**
 * @brief 清除所有姿态约束
 */
void ArmController::clear_constraints() {
    _constraints_ = moveit_msgs::msg::Constraints();
    _arm_.clearPathConstraints();
    RCLCPP_INFO(_node_->get_logger(), "已清除所有姿态约束");
}

/**
 * @brief 判断当前是否有异步规划或执行正在进行
 * @return 如果有异步规划或执行正在进行，则返回 true；否则返回 false
 */
bool ArmController::is_planning_or_executing() const {
    return _is_planning_or_executing_;
}

/**
 * @brief 取消当前的异步规划或执行
 * @return 是否成功取消异步任务的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode ArmController::cancel_async() {
    if(!_is_planning_or_executing_) {
        RCLCPP_INFO(_node_->get_logger(), "当前没有正在进行的异步任务");
        return ErrorCode::SUCCESS;
    }

    RCLCPP_INFO(_node_->get_logger(), "正在取消异步任务...");
    _arm_.stop();

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    _is_planning_or_executing_ = false;
    RCLCPP_INFO(_node_->get_logger(), "异步任务已取消");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 将 Roll-Pitch-Yaw 角转换为四元数
 * @param roll 滚转角（绕 X 轴旋转）
 * @param pitch 俯仰角（绕 Y 轴旋转）
 * @param yaw 偏航角（绕 Z 轴旋转）
 * @return 转换后的四元数
 */
geometry_msgs::msg::Quaternion ArmController::rpy_to_quaternion(double roll, double pitch, double yaw) {
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
geometry_msgs::msg::Pose ArmController::rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = rpy_to_quaternion(roll, pitch, yaw);

    return pose;
}

/**
 * @brief 获取规划组名称
 * @return 规划组名称
 */
const std::string& ArmController::get_arm_name() const {
    return _arm_.getName();
}

/**
 * @brief 获取当前关节值
 * @return 当前关节值的向量
 */
std::vector<double> ArmController::get_current_joints() const {
    return _arm_.getCurrentJointValues();
}

/**
 * @brief 获取当前末端执行器的位姿（底座坐标系）
 * @return 当前末端执行器的位姿
 */
geometry_msgs::msg::Pose ArmController::get_current_pose() const {
    return _arm_.getCurrentPose().pose;
}

/**
 * @brief 获取当前机械臂的关节名称
 * @return 当前机械臂的关节名称的向量
 */
std::vector<std::string> ArmController::get_current_link_names() const {
    return _arm_.getJointNames();
}

} /* namespace dm_arm */

// ! ========================= 私 有 函 数 实 现 ========================= ! //


