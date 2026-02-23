#include "dm_arm_controller/tasks_manager.hpp"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/predicate_filter.h>

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

namespace mtc = moveit::task_constructor;

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

namespace dm_arm {

/**
 * @brief 任务管理器构造函数
 * @param node ROS 2 节点指针
 * @param arm_group_name 机械臂规划组名称
 * @param eef_group_name 末端执行器规划组名称
 */
TasksManager::TasksManager(rclcpp::Node::SharedPtr node, const std::string& arm_group_name, const std::string& eef_group_name)
    : _node_(std::move(node)), _arm_group_name_(arm_group_name), _eef_group_name_(eef_group_name) {

    // 创建控制器实例
    _arm_ = std::make_shared<ArmController>(_node_, _arm_group_name_);
    _eef_ = std::make_shared<TwoFingerGripper>(_node_, _eef_group_name_);
    _arm_->attach_eef(_eef_);

    // 从机器人模型动态获取末端执行器坐标系
    auto temp_task = std::make_shared<mtc::Task>();
    temp_task->loadRobotModel(_node_);
    auto robot_model = temp_task->getRobotModel();
    auto eef_jmg = robot_model->getJointModelGroup(_eef_group_name_);
    if(eef_jmg && eef_jmg->isEndEffector()) {
        _hand_frame_ = eef_jmg->getEndEffectorParentGroup().second;
    }
    else {
        auto arm_jmg = robot_model->getJointModelGroup(_arm_group_name_);
        if(arm_jmg && arm_jmg->getOnlyOneEndEffectorTip()) {
            _hand_frame_ = arm_jmg->getOnlyOneEndEffectorTip()->getName();
        }
    }
    if(_hand_frame_.empty()) {
        RCLCPP_ERROR(_node_->get_logger(),
            "无法自动获取末端 TCP 坐标系，请检查 SRDF 中 end_effector 定义");
    }

    // 声明参数
    _node_->declare_parameter("motion_planning.max_velocity_scaling_factor", 0.1);
    _node_->declare_parameter("motion_planning.max_acceleration_scaling_factor", 0.1);
    _node_->declare_parameter("motion_planning.planning_time", 5.0);
    _node_->declare_parameter("decartes.eef_step", 0.01);
    _node_->declare_parameter("tasks_manager.max_retries", 3);
    _node_->declare_parameter("tasks_manager.max_solutions", 10);

    _max_retries_ = static_cast<int>(_node_->get_parameter("tasks_manager.max_retries").as_int());
    _max_solutions_ = static_cast<size_t>(_node_->get_parameter("tasks_manager.max_solutions").as_int());

    // 采样规划器 (OMPL)
    _sampling_solver_ = std::make_shared<mtc::solvers::PipelinePlanner>(_node_);

    // 笛卡尔规划器
    _cartesian_solver_ = std::make_shared<mtc::solvers::CartesianPath>();
    _cartesian_solver_->setMaxVelocityScalingFactor(
        _node_->get_parameter("motion_planning.max_velocity_scaling_factor").as_double());
    _cartesian_solver_->setMaxAccelerationScalingFactor(
        _node_->get_parameter("motion_planning.max_acceleration_scaling_factor").as_double());
    _cartesian_solver_->setStepSize(
        _node_->get_parameter("decartes.eef_step").as_double());

    // 关节插值规划器
    _joint_interp_solver_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    RCLCPP_INFO(_node_->get_logger(), "任务管理器初始化完成 | arm=%s, eef=%s, tcp_frame=%s",
        _arm_group_name_.c_str(), _eef_group_name_.c_str(), _hand_frame_.c_str());
}

/**
 * @brief 任务管理器析构函数
 */
TasksManager::~TasksManager() {
    cancel();
}

/**
 * @brief 创建任务组
 * @param name 任务组名称
 * @param mode 任务执行模式
 */
int TasksManager::create_task_group(const std::string& name, ExecutionMode mode) {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    TaskGroup group;
    group.group_id = _next_group_id_++;
    group.name = name;
    group.mode = mode;
    _task_groups_[group.group_id] = std::move(group);

    RCLCPP_INFO(_node_->get_logger(), "已创建任务组 ID=%d, 名称='%s', 模式=%s",
        group.group_id, name.c_str(),
        mode == ExecutionMode::BY_ID ? "BY_ID" : "BY_SHORTEST_PATH");
    return _task_groups_.rbegin()->first;
}

/**
 * @brief 移除任务组
 * @param group_id 任务组 ID
 */
bool TasksManager::remove_task_group(int group_id) {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    return _task_groups_.erase(group_id) > 0;
}

/**
 * @brief 清空所有任务组
 */
void TasksManager::clear_task_groups() {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    _task_groups_.clear();
}

/**
 * @brief 向任务组添加或更新任务
 * @param group_id 任务组 ID
 * @param task 任务描述符
 */
bool TasksManager::add_task(int group_id, const TaskDescriptor& task) {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    auto git = _task_groups_.find(group_id);
    if(git == _task_groups_.end()) {
        RCLCPP_ERROR(_node_->get_logger(), "未找到任务组 ID=%d", group_id);
        return false;
    }

    auto& tasks = git->second.tasks;
    auto it = std::find_if(tasks.begin(), tasks.end(), [&](const TaskDescriptor& t) { return t.id == task.id; });
    if(it != tasks.end()) {
        *it = task;
    }
    else {
        tasks.push_back(task);
    }

    RCLCPP_INFO(_node_->get_logger(), "已添加任务 ID=%d 到组 ID=%d, 类型=%d",
        task.id, group_id, static_cast<int>(task.type));
    return true;
}

/**
 * @brief 从任务组移除任务
 * @param group_id 任务组 ID
 * @param task_id 任务 ID
 */
bool TasksManager::remove_task(int group_id, int task_id) {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    auto git = _task_groups_.find(group_id);
    if(git == _task_groups_.end()) return false;

    auto& tasks = git->second.tasks;
    auto it = std::find_if(tasks.begin(), tasks.end(), [task_id](const TaskDescriptor& t) { return t.id == task_id; });
    if(it == tasks.end()) return false;
    tasks.erase(it);

    return true;
}

/**
 * @brief 获取任务组中的任务描述符
 * @param group_id 任务组 ID
 * @param task_id 任务 ID
 * @return 指向任务描述符的指针，若未找到则返回 nullptr
 */
const TaskDescriptor* TasksManager::get_task(int group_id, int task_id) const {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    auto git = _task_groups_.find(group_id);
    if(git == _task_groups_.end()) return nullptr;

    for(const auto& t : git->second.tasks) {
        if(t.id == task_id) return &t;
    }

    return nullptr;
}

/**
 * @brief 清空任务组中的所有任务
 * @param group_id 任务组 ID
 */
void TasksManager::clear_tasks(int group_id) {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    auto git = _task_groups_.find(group_id);
    if(git != _task_groups_.end()) {
        git->second.tasks.clear();
    }
}

/**
 * @brief 执行单个任务
 * @param group_id 任务组 ID
 * @param task_id 任务 ID
 */
ErrorCode TasksManager::execute_single_task(int group_id, int task_id) {
    if(_is_busy_) {
        RCLCPP_WARN(_node_->get_logger(), "当前忙碌，无法执行任务 group=%d, task=%d", group_id, task_id);
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    std::lock_guard<std::mutex> lock(_tasks_mutex_);

    TaskDescriptor desc;
    auto git = _task_groups_.find(group_id);
    if(git == _task_groups_.end()) {
        RCLCPP_ERROR(_node_->get_logger(), "未找到任务组 ID=%d", group_id);
        return ErrorCode::PLANNING_FAILED;
    }

    auto it = std::find_if(git->second.tasks.begin(), git->second.tasks.end(),
        [task_id](const TaskDescriptor& t) { return t.id == task_id; });
    if(it == git->second.tasks.end()) {
        RCLCPP_ERROR(_node_->get_logger(), "未找到任务 ID=%d (组 ID=%d)", task_id, group_id);
        return ErrorCode::PLANNING_FAILED;
    }
    desc = *it;

    _is_busy_ = true;
    _cancel_requested_ = false;

    ErrorCode result = execute_task_internal(desc);

    // 写回状态
    if(result == ErrorCode::SUCCESS) {
        update_task_status(group_id, task_id, TaskStatus::SUCCEEDED, ErrorCode::SUCCESS, "执行成功");
    }
    else {
        update_task_status(group_id, task_id, TaskStatus::FAILED, result, "执行失败");
    }

    _is_busy_ = false;
    return result;
}

/**
 * @brief 内部执行任务函数，包含重试机制
 * @param desc 任务描述符
 */
ErrorCode TasksManager::execute_task_internal(const TaskDescriptor& desc) {
    ErrorCode result = ErrorCode::PLANNING_FAILED;

    for(int attempt = 0; attempt < _max_retries_; ++attempt) {
        if(_cancel_requested_) {
            publish_feedback(desc.id, TaskStatus::CANCELLED, "cancel", "用户取消", 1.0);
            return ErrorCode::EXECUTION_FAILED;
        }

        RCLCPP_INFO(_node_->get_logger(), "执行任务 ID=%d, 尝试 %d/%d",
            desc.id, attempt + 1, _max_retries_);
        publish_feedback(desc.id, TaskStatus::PLANNING, "init", "开始规划", 0.0);

        mtc::TaskPtr mtc_task;
        switch(desc.type) {
            case TaskType::MOVE_TO_HOME:
                mtc_task = build_move_to_home_task();
                break;
            case TaskType::PICK_AND_PLACE:
                mtc_task = build_pick_and_place_task(desc);
                break;
            default:
                RCLCPP_WARN(_node_->get_logger(), "未知任务类型 %d", static_cast<int>(desc.type));
                return ErrorCode::PLANNING_FAILED;
        }

        if(!mtc_task) {
            RCLCPP_ERROR(_node_->get_logger(), "构建 MTC Task 失败");
            continue;
        }

        // 规划
        result = plan_mtc_task(*mtc_task, _max_solutions_);
        if(result != ErrorCode::SUCCESS) {
            RCLCPP_WARN(_node_->get_logger(), "规划失败 (尝试 %d/%d)", attempt + 1, _max_retries_);
            publish_feedback(desc.id, TaskStatus::FAILED, "plan", "规划失败，将重试",
                static_cast<double>(attempt + 1) / _max_retries_);
            continue;
        }

        // 执行
        publish_feedback(desc.id, TaskStatus::EXECUTING, "execute", "开始执行轨迹", 0.5);

        result = execute_mtc_task(*mtc_task);
        if(result == ErrorCode::SUCCESS) {
            publish_feedback(desc.id, TaskStatus::SUCCEEDED, "done", "任务完成", 1.0);
            return ErrorCode::SUCCESS;
        }

        RCLCPP_WARN(_node_->get_logger(), "执行失败 (尝试 %d/%d)", attempt + 1, _max_retries_);
        publish_feedback(desc.id, TaskStatus::FAILED, "execute", "执行失败，将重试",
            static_cast<double>(attempt + 1) / _max_retries_);
    }

    publish_feedback(desc.id, TaskStatus::FAILED, "final", "任务最终失败", 1.0);
    return result;
}

/**
 * @brief 执行任务组
 * @param group_id 任务组 ID
 */
ErrorCode TasksManager::execute_task_group(int group_id) {
    if(_is_busy_) {
        RCLCPP_WARN(_node_->get_logger(), "当前忙碌，无法执行任务组 ID=%d", group_id);
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    _is_busy_ = true;
    _cancel_requested_ = false;

    // 拷贝任务组
    TaskGroup group;
    {
        std::lock_guard<std::mutex> lock(_tasks_mutex_);
        auto it = _task_groups_.find(group_id);
        if(it == _task_groups_.end()) {
            RCLCPP_ERROR(_node_->get_logger(), "未找到任务组 ID=%d", group_id);
            _is_busy_ = false;
            return ErrorCode::PLANNING_FAILED;
        }
        group = it->second;
    }

    if(group.tasks.empty()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 没有任务", group.name.c_str());
        _is_busy_ = false;
        return ErrorCode::SUCCESS;
    }

    // 根据执行模式排序
    std::vector<int> ordered_ids;
    switch(group.mode) {
        case ExecutionMode::BY_ID:
            ordered_ids = order_by_id(group);
            break;
        case ExecutionMode::BY_SHORTEST_PATH:
            ordered_ids = order_by_shortest_path(group);
            break;
    }

    RCLCPP_INFO(_node_->get_logger(), "开始执行任务组 '%s' (ID=%d), 共 %zu 个任务, 模式=%s",
        group.name.c_str(), group_id, ordered_ids.size(),
        group.mode == ExecutionMode::BY_ID ? "BY_ID" : "BY_SHORTEST_PATH");

    // 打印执行顺序
    std::string order_str;
    for(size_t i = 0; i < ordered_ids.size(); ++i) {
        order_str += std::to_string(ordered_ids[i]);
        if(i + 1 < ordered_ids.size()) order_str += " -> ";
    }
    RCLCPP_INFO(_node_->get_logger(), "执行顺序: %s", order_str.c_str());

    // 逐任务执行
    std::vector<int> executed_ids;
    ErrorCode group_result = ErrorCode::SUCCESS;

    for(size_t i = 0; i < ordered_ids.size(); ++i) {
        int tid = ordered_ids[i];

        if(_cancel_requested_) {
            RCLCPP_WARN(_node_->get_logger(), "任务组执行被取消");
            group_result = ErrorCode::EXECUTION_FAILED;
            break;
        }

        RCLCPP_INFO(_node_->get_logger(), "── 执行任务 %zu/%zu (ID=%d) ──",
            i + 1, ordered_ids.size(), tid);

        auto it = std::find_if(group.tasks.begin(), group.tasks.end(),
            [tid](const TaskDescriptor& t) { return t.id == tid; });
        if(it == group.tasks.end()) continue;

        ErrorCode ec = execute_task_internal(*it);

        if(ec == ErrorCode::SUCCESS) {
            update_task_status(group_id, tid, TaskStatus::SUCCEEDED, ErrorCode::SUCCESS, "执行成功");
            executed_ids.push_back(tid);
        }
        else {
            update_task_status(group_id, tid, TaskStatus::FAILED, ec, "执行失败");
            RCLCPP_ERROR(_node_->get_logger(), "任务 ID=%d 失败，启动回滚策略", tid);
            group_result = ec;

            rollback(group_id, executed_ids);
            break;
        }
    }

    if(group_result == ErrorCode::SUCCESS) {
        RCLCPP_INFO(_node_->get_logger(), "任务组 '%s' 全部执行成功", group.name.c_str());
    }

    _is_busy_ = false;
    return group_result;
}

/**
 * @brief 设置任务执行反馈回调函数
 * @param cb 反馈回调函数，参数为 ExecutionFeedback 结构体
 */
void TasksManager::set_feedback_callback(FeedbackCallback cb) {
    _feedback_cb_ = std::move(cb);
}

/**
 * @brief 取消当前正在执行的任务或任务组
 */
void TasksManager::cancel() {
    _cancel_requested_ = true;
}

/**
 * @brief 检查任务管理器是否正在执行任务
 * @return 如果正在执行任务则返回 true，否则返回 false
 */
bool TasksManager::is_busy() const {
    return _is_busy_;
}

/**
 * @brief 设置任务执行的最大重试次数
 * @param retries 最大重试次数，必须大于 0
 */
void TasksManager::set_max_retries(int retries) {
    _max_retries_ = std::max(1, retries);
}

/**
 * @brief 设置任务执行的最大解算方案数量
 * @param n 最大解算方案数量，必须大于 0
 */
void TasksManager::set_max_solutions(size_t n) {
    _max_solutions_ = std::max(static_cast<size_t>(1), n);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //


/**
 * @brief 发布任务执行反馈
 * @param task_id 任务 ID
 * @param status 任务状态
 * @param stage 当前执行阶段名称
 * @param msg 相关消息
 * @param progress 执行进度（0.0 ~ 1.0）
 */
void TasksManager::publish_feedback(int task_id, TaskStatus status, const std::string& stage, const std::string& msg, double progress) {
    if(_feedback_cb_) {
        ExecutionFeedback fb;
        fb.task_id = task_id;
        fb.status = status;
        fb.stage_name = stage;
        fb.message = msg;
        fb.progress = progress;
        _feedback_cb_(fb);
    }

    RCLCPP_INFO(_node_->get_logger(), "反馈: task=%d, status=%d, stage='%s', msg='%s', progress=%.1f%%",
        task_id, static_cast<int>(status), stage.c_str(), msg.c_str(), progress * 100.0);
}

/**
 * @brief 更新任务状态
 * @param group_id 任务组 ID
 * @param task_id 任务 ID
 * @param status 任务状态
 * @param ec 错误代码
 * @param msg 相关消息
 */
void TasksManager::update_task_status(int group_id, int task_id, TaskStatus status, ErrorCode ec, const std::string& msg) {
    std::lock_guard<std::mutex> lock(_tasks_mutex_);
    auto git = _task_groups_.find(group_id);
    if(git == _task_groups_.end()) return;
    for(auto& t : git->second.tasks) {
        if(t.id == task_id) {
            t.status = status;
            t.error_code = ec;
            if(!msg.empty()) t.message = msg;
            break;
        }
    }
}

/**
 * @brief 构建移动到初始位的 MTC 任务
 * @return 构建成功返回指向 MTC 任务的智能指针，失败返回 nullptr
 */
mtc::TaskPtr TasksManager::build_move_to_home_task() {
    auto task = std::make_shared<mtc::Task>();
    task->stages()->setName("MoveToHome");
    task->loadRobotModel(_node_);

    // 设置 Task 属性
    task->setProperty("group", _arm_group_name_);
    task->setProperty("eef", _eef_group_name_);
    task->setProperty("hand", _eef_group_name_);
    task->setProperty("ik_frame", _hand_frame_);

    // Stage 1: 获取当前状态
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("当前状态");
        task->add(std::move(stage));
    }

    // Stage 2: 移动到 Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("移动到 Home", _sampling_solver_);
        stage->setGroup(_arm_group_name_);
        stage->setGoal("home");
        task->add(std::move(stage));
    }

    try {
        task->init();
    }
    catch(const mtc::InitStageException& e) {
        RCLCPP_ERROR(_node_->get_logger(), "MoveToHome Task 初始化失败: %s", e.what());
        return nullptr;
    }

    return task;
}

/**
 * @brief 构建抓取和放置的 MTC 任务
 * @param desc 任务描述符，包含抓取和放置的相关信息
 */
mtc::TaskPtr TasksManager::build_pick_and_place_task(const TaskDescriptor& desc) {
    auto task = std::make_shared<mtc::Task>();
    task->stages()->setName("PickAndPlace");
    task->loadRobotModel(_node_);

    // 设置全局属性
    task->setProperty("group", _arm_group_name_);
    task->setProperty("eef", _eef_group_name_);
    task->setProperty("hand", _eef_group_name_);
    task->setProperty("hand_grasping_frame", _hand_frame_);
    task->setProperty("ik_frame", _hand_frame_);

    // Stage 1: Current State
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("当前状态");

        // 验证物体未被附着
        auto applicability_filter =
            std::make_unique<mtc::stages::PredicateFilter>("前置检查", std::move(stage));
        applicability_filter->setPredicate(
            [object_id = desc.object_id](const mtc::SolutionBase& s, std::string& comment) {
                if(s.start()->scene()->getCurrentState().hasAttachedBody(object_id)) {
                    comment = "物体 '" + object_id + "' 已处于附着状态，无法抓取";
                    return false;
                }
                return true;
            });
        task->add(std::move(applicability_filter));
    }

    // Stage 2: 打开夹爪
    mtc::Stage* open_hand_ptr = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("打开夹爪", _joint_interp_solver_);
        stage->setGroup(_eef_group_name_);
        stage->setGoal("open");
        open_hand_ptr = stage.get();
        task->add(std::move(stage));
    }

    // Stage 3: 连接到抓取位
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "移动到抓取位",
            mtc::stages::Connect::GroupPlannerVector{ {_arm_group_name_, _sampling_solver_} });
        stage->setTimeout(_node_->get_parameter("motion_planning.planning_time").as_double());
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task->add(std::move(stage));
    }

    // Stage 4: Pick (SerialContainer)
    mtc::Stage* pick_stage_ptr = nullptr;
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("抓取物体");
        task->properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

        // Stage 4.1: 接近物体
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("接近物体", _cartesian_solver_);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", _hand_frame_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(desc.approach_distance * 0.5, desc.approach_distance);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = _hand_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // Stage 4.2: 生成抓取位姿
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("生成抓取位姿");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject(desc.object_id);
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(open_hand_ptr);

            // 计算 IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("抓取位姿 IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(_hand_frame_);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));
        }

        // Stage 4.3: 允许碰撞 (手-物体)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("允许碰撞(手-物体)");
            stage->allowCollisions(
                desc.object_id,
                task->getRobotModel()->getJointModelGroup(_eef_group_name_)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        // Stage 4.4: 关闭夹爪
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("关闭夹爪", _joint_interp_solver_);
            stage->setGroup(_eef_group_name_);
            stage->setGoal("close");
            grasp->insert(std::move(stage));
        }

        // Stage 4.5: 附着物体
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("附着物体");
            stage->attachObject(desc.object_id, _hand_frame_);
            grasp->insert(std::move(stage));
        }

        // Stage 4.6: 允许碰撞 (物体-支撑面)
        if(!desc.support_surface.empty()) {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("允许碰撞(物体-支撑面)");
            stage->allowCollisions({ desc.object_id }, { desc.support_surface }, true);
            grasp->insert(std::move(stage));
        }

        // Stage 4.7: 抬升物体
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("抬升物体", _cartesian_solver_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(desc.lift_distance * 0.5, desc.lift_distance);
            stage->setIKFrame(_hand_frame_);
            stage->properties().set("marker_ns", "lift_object");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // Stage 4.8: 禁止碰撞 (物体-支撑面)
        if(!desc.support_surface.empty()) {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("禁止碰撞(物体-支撑面)");
            stage->allowCollisions({ desc.object_id }, { desc.support_surface }, false);
            grasp->insert(std::move(stage));
        }

        pick_stage_ptr = grasp.get();
        task->add(std::move(grasp));
    }

    // Stage 5: 连接到放置位
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "移动到放置位",
            mtc::stages::Connect::GroupPlannerVector{ {_arm_group_name_, _sampling_solver_} });
        stage->setTimeout(_node_->get_parameter("motion_planning.planning_time").as_double());
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task->add(std::move(stage));
    }

    // Stage 6: Place (SerialContainer)
    {
        auto place = std::make_unique<mtc::SerialContainer>("放置物体");
        task->properties().exposeTo(place->properties(), { "eef", "hand", "group" });
        place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group" });

        // Stage 6.1: 降低物体
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("降低物体", _cartesian_solver_);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", _hand_frame_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.03, 0.13);

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "world";
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        // Stage 6.2: 生成放置位姿
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("生成放置位姿");
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(desc.object_id);

            geometry_msgs::msg::PoseStamped place_pose_stamped;
            place_pose_stamped.header.frame_id = "world";
            place_pose_stamped.pose = desc.place_pose;
            stage->setPose(place_pose_stamped);
            stage->setMonitoredStage(pick_stage_ptr);

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("放置位姿 IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setIKFrame(_hand_frame_);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        // Stage 6.3: 打开夹爪
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("打开夹爪(放置)", _joint_interp_solver_);
            stage->setGroup(_eef_group_name_);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }

        // Stage 6.4: 禁止碰撞 (手-物体)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("禁止碰撞(手-物体)");
            stage->allowCollisions(
                desc.object_id,
                *task->getRobotModel()->getJointModelGroup(_eef_group_name_),
                false);
            place->insert(std::move(stage));
        }

        // Stage 6.5: 分离物体
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("分离物体");
            stage->detachObject(desc.object_id, _hand_frame_);
            place->insert(std::move(stage));
        }

        // Stage 6.6: 撤退
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("撤退", _cartesian_solver_);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(desc.retreat_distance * 0.5, desc.retreat_distance);
            stage->setIKFrame(_hand_frame_);
            stage->properties().set("marker_ns", "retreat");

            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = _hand_frame_;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        task->add(std::move(place));
    }

    // Stage 7: 回到 Home
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("回到 Home", _sampling_solver_);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("home");
        task->add(std::move(stage));
    }

    // 初始化
    try {
        task->init();
    }
    catch(const mtc::InitStageException& e) {
        RCLCPP_ERROR(_node_->get_logger(), "PickAndPlace Task 初始化失败: %s", e.what());
        return nullptr;
    }

    return task;
}

/**
 * @brief 规划 MTC 任务
 * @param task MTC 任务对象
 * @param max_solutions 最大解算方案数量
 * @return 规划结果的错误代码
 */
ErrorCode TasksManager::plan_mtc_task(mtc::Task& task, size_t max_solutions) {
    RCLCPP_INFO(_node_->get_logger(), "MTC 开始规划 (max_solutions=%zu)...", max_solutions);

    auto result = task.plan(max_solutions);
    if(result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(_node_->get_logger(), "MTC 规划失败, 错误码=%d", result.val);
        task.printState();
        task.explainFailure();
        return ErrorCode::PLANNING_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "MTC 规划成功, 找到 %zu 个解", task.numSolutions());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行 MTC 任务
 * @param task MTC 任务对象，必须已经成功规划
 * @return 执行结果的错误代码
 */
ErrorCode TasksManager::execute_mtc_task(mtc::Task& task) {
    if(task.numSolutions() == 0) {
        RCLCPP_ERROR(_node_->get_logger(), "没有可用的解来执行");
        return ErrorCode::PLANNING_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "MTC 开始执行最优解...");

    auto result = task.execute(*task.solutions().front());
    if(result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(_node_->get_logger(), "MTC 执行失败, 错误码=%d", result.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    RCLCPP_INFO(_node_->get_logger(), "MTC 执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 根据任务 ID 顺序排序任务
 * @param group 任务组
 * @return 按照任务 ID 升序排序的任务 ID 列表
 */
std::vector<int> TasksManager::order_by_id(const TaskGroup& group) const {
    std::vector<int> ids;
    ids.reserve(group.tasks.size());
    for(const auto& t : group.tasks) {
        ids.push_back(t.id);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

/**
 * @brief 根据末端位姿的最短路径排序任务，使用贪心最近邻算法
 * @param group 任务组
 * @return 按照执行顺序排序的任务 ID 列表
 */
std::vector<int> TasksManager::order_by_shortest_path(const TaskGroup& group) const {
    if(group.tasks.size() <= 1) {
        std::vector<int> ids;
        for(const auto& t : group.tasks) ids.push_back(t.id);
        return ids;
    }

    // 获取当前末端位姿作为起始点
    geometry_msgs::msg::Pose ref_pose = _arm_->get_current_pose();

    // 准备候选任务（按值拷贝，贪心过程中逐个移除）
    std::vector<TaskDescriptor> remaining = group.tasks;

    std::vector<int> ordered;
    ordered.reserve(remaining.size());

    // 贪心最近邻算法
    while(!remaining.empty()) {
        double min_dist = std::numeric_limits<double>::max();
        size_t min_idx = 0;

        for(size_t i = 0; i < remaining.size(); ++i) {
            geometry_msgs::msg::Pose target = get_task_target_pose(remaining[i]);
            double dist = pose_distance(ref_pose, target);
            if(dist < min_dist) {
                min_dist = dist;
                min_idx = i;
            }
        }

        ordered.push_back(remaining[min_idx].id);
        ref_pose = get_task_target_pose(remaining[min_idx]);
        remaining.erase(remaining.begin() + static_cast<long>(min_idx));
    }

    return ordered;
}

/**
 * @brief 计算两个位姿之间的欧氏距离（仅位置部分）
 * @param a 位姿 A
 * @param b 位姿 B
 * @return 两个位姿之间的距离
 */
double TasksManager::pose_distance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 获取任务的目标位姿，用于最近邻排序
 * @param desc 任务描述符
 * @return 任务的目标位姿
 */
geometry_msgs::msg::Pose TasksManager::get_task_target_pose(const TaskDescriptor& desc) const {
    switch(desc.type) {
        case TaskType::PICK_AND_PLACE:
            return desc.grasp_pose;
        case TaskType::MOVE_TO_HOME:
        default: {
            // Home 的代表位姿使用默认零点
            geometry_msgs::msg::Pose p;
            p.orientation.w = 1.0;
            return p;
        }
    }
}

/**
 * @brief 回滚已执行的任务
 * @param group_id 任务组 ID
 * @param executed_ids 已成功执行的任务 ID 列表
 */
void TasksManager::rollback(int group_id, const std::vector<int>& executed_ids) {
    if(executed_ids.empty()) return;

    RCLCPP_WARN(_node_->get_logger(), "开始回滚 %zu 个已执行的任务", executed_ids.size());
    publish_feedback(-1, TaskStatus::ROLLED_BACK, "rollback", "启动回滚策略", 0.0);

    // 回滚策略：优先 MTC 回到 Home，失败时降级使用控制器直接复位
    bool mtc_ok = false;
    auto home_task = build_move_to_home_task();
    if(home_task) {
        ErrorCode ec = plan_mtc_task(*home_task, 5);
        if(ec == ErrorCode::SUCCESS) {
            ec = execute_mtc_task(*home_task);
            if(ec == ErrorCode::SUCCESS) {
                RCLCPP_INFO(_node_->get_logger(), "MTC 回滚成功：已回到 Home 位姿");
                mtc_ok = true;
            }
        }
    }

    // MTC 回滚失败时，使用 ArmController + EefController 直接复位
    if(!mtc_ok) {
        RCLCPP_WARN(_node_->get_logger(), "MTC 回滚失败，使用控制器直接复位");
        _eef_->open();
        _arm_->home();
    }

    // 更新所有已执行任务的状态为 ROLLED_BACK
    for(int tid : executed_ids) {
        update_task_status(group_id, tid, TaskStatus::ROLLED_BACK, ErrorCode::SUCCESS, "已回滚");
    }

    publish_feedback(-1, TaskStatus::ROLLED_BACK, "rollback", "回滚完成", 1.0);
}

} /* namespace dm_arm */
