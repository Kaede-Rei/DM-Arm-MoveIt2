#include "dm_arm_controller/tasks_manager.hpp"

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

namespace dm_arm {

/**
 * @brief TasksManager 构造函数：初始化成员变量
 * @param node 共享指针，指向 ROS 2 节点
 * @param arm 共享指针，指向 ArmController 实例
 * @param eef 共享指针，指向 EndEffector 实例
 */
TasksManager::TasksManager(rclcpp::Node::SharedPtr node, std::shared_ptr<ArmController> arm, std::shared_ptr<EndEffector> eef)
    : _node_(std::move(node)), _arm_(std::move(arm)), _eef_(std::move(eef)) {
    _arm_name_ = _arm_->get_arm_name();
    _eef_name_ = _eef_->get_eef_name();
}

/**
 * @brief 创建任务组
 * @param group_name 任务组名称
 * @param sort_type 任务排序方式，默认为按 ID 排序
 * @return 创建结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::create_task_group(const std::string& group_name, SortType sort_type) {
    if(_task_groups_.find(group_name) != _task_groups_.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 已存在，无法创建", group_name.c_str());
        return ErrorCode::TASK_GROUP_EXISTS;
    }

    TaskGroup new_group;
    new_group.sort_type = sort_type;
    new_group.tasks.clear();

    _task_groups_[group_name] = std::move(new_group);
    RCLCPP_INFO(_node_->get_logger(), "成功创建任务组 '%s'", group_name.c_str());

    return ErrorCode::SUCCESS;
}

/**
 * @brief 删除任务组
 * @param group_name 任务组名称
 * @return 删除结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::delete_task_group(const std::string& group_name) {
    auto it = _task_groups_.find(group_name);
    if(it == _task_groups_.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 不存在，无法删除", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    _task_groups_.erase(group_name);
    RCLCPP_INFO(_node_->get_logger(), "成功删除任务组 '%s'", group_name.c_str());

    return ErrorCode::SUCCESS;
}

/**
 * @brief 清空任务组中的所有任务
 * @param group_name 任务组名称
 * @return 清空结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::clear_task_group(const std::string& group_name) {
    auto it = _task_groups_.find(group_name);
    if(it == _task_groups_.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 不存在，无法清空", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    it->second.tasks.clear();

    RCLCPP_INFO(_node_->get_logger(), "成功清空任务组 '%s'", group_name.c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 向任务组中添加任务
 * @param group_name 任务组名称
 * @param id 任务 ID，必须在任务组内唯一
 * @param task_type 任务类型，默认为 TaskType::NONE
 * @param task_description 任务描述，默认为空字符串
 * @return 添加结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::add_task(const std::string& group_name, int id, TaskType task_type, const std::string& task_description) {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 不存在，无法添加任务", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    Task new_task;
    new_task.desc = task_description;
    new_task.type = task_type;

    if(task_group->second.tasks.find(id) != task_group->second.tasks.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 中已存在任务 ID %d，无法添加", group_name.c_str(), id);
        return ErrorCode::TASK_EXISTS;
    }

    task_group->second.tasks[id] = std::move(new_task);
    RCLCPP_INFO(_node_->get_logger(), "成功向任务组 '%s' 添加任务 ID %d", group_name.c_str(), id);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 从任务组中删除任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 删除结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::delete_task(const std::string& group_name, int id) {
    ErrorCode error_code;
    Task* task = find_task(group_name, id, error_code);
    if(task == nullptr) {
        return error_code;
    }

    auto task_group = _task_groups_.find(group_name);
    task_group->second.tasks.erase(id);
    RCLCPP_INFO(_node_->get_logger(), "成功从任务组 '%s' 删除任务 ID %d", group_name.c_str(), id);

    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置任务的目标
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param target 任务目标，可以是位姿、位置或姿态等
 * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::set_task_target(const std::string& group_name, int id, const TargetVariant& target) {
    ErrorCode error_code;
    Task* task = find_task(group_name, id, error_code);
    if(task == nullptr) {
        return error_code;
    }

    task->target = target;
    RCLCPP_INFO(_node_->get_logger(), "成功设置任务组 '%s'中任务 ID %d 的目标", group_name.c_str(), id);

    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行指定任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::execute_task(const std::string& group_name, int id) {
    ErrorCode error_code;
    Task* task = find_task(group_name, id, error_code);
    if(task == nullptr) {
        return error_code;
    }

    return execute_task(*task);
}

/**
 * @brief 执行指定任务
 * @param task 任务对象
 * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::execute_task(Task& task) {
    RCLCPP_INFO(_node_->get_logger(), "开始执行任务: %s", task.desc.c_str());
    RCLCPP_INFO(_node_->get_logger(), "任务描述: %s", task.desc.c_str());

    if(task.type == TaskType::NONE) {
        _arm_->set_target(task.target);
        _arm_->plan_and_execute();
        RCLCPP_INFO(_node_->get_logger(), "无特定任务，正在移动到指定目标...");
    }
    else if(task.type == TaskType::PICK) {
        // TODO: 实现 PICK 任务的具体逻辑，例如控制末端执行器抓取物体等

        RCLCPP_INFO(_node_->get_logger(), "正在执行 PICK 任务...");
    }

    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行任务组中的所有任务
 * @param group_name 任务组名称
 * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::execute_task_group(const std::string& group_name) {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 不存在，无法执行", group_name.c_str());
        return ErrorCode::TASK_GROUP_NOT_FOUND;
    }

    RCLCPP_INFO(_node_->get_logger(), "开始执行任务组 '%s'", group_name.c_str());

    sort_tasks(task_group->second);
    for(auto& task : task_group->second.sorted_tasks) {
        ErrorCode err_code = execute_task(task);
        if(err_code != ErrorCode::SUCCESS) {
            RCLCPP_WARN(_node_->get_logger(), "执行任务组 '%s' 中任务 ID %d 失败，错误码：%s", group_name.c_str(), task.id, err_to_string(err_code).c_str());
            return err_code;
        }
    }

    return ErrorCode::SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 查找任务组中的任务
 * @param group_name 任务组名称
 * @param id 任务 ID
 * @param error_code 输出参数，返回查找结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 * @return 指向找到的任务的指针，如果未找到则返回 nullptr
 */
Task* TasksManager::find_task(const std::string& group_name, int id, ErrorCode& error_code) {
    auto task_group = _task_groups_.find(group_name);
    if(task_group == _task_groups_.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 不存在", group_name.c_str());
        error_code = ErrorCode::TASK_GROUP_NOT_FOUND;
        return nullptr;
    }

    auto task = task_group->second.tasks.find(id);
    if(task == task_group->second.tasks.end()) {
        RCLCPP_WARN(_node_->get_logger(), "任务组 '%s' 中不存在任务 ID %d", group_name.c_str(), id);
        error_code = ErrorCode::TASK_NOT_FOUND;
        return nullptr;
    }

    return &(task->second);
}

/**
 * @brief 对任务组中的任务进行排序
 * @param task_group 需要排序的任务组
 * @return 排序结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
 */
ErrorCode TasksManager::sort_tasks(TaskGroup& task_group) {
    task_group.sorted_tasks.clear();
    for(auto& [id, task] : task_group.tasks) {
        task_group.sorted_tasks.push_back(task);
        task_group.sorted_tasks.back().id = id;
    }

    if(task_group.sort_type == SortType::ID) {
        RCLCPP_INFO(_node_->get_logger(), "任务组已按 ID 排序");
    }
    else if(task_group.sort_type == SortType::DIST) {
        // TODO: 实现按距离排序的逻辑
        RCLCPP_WARN(_node_->get_logger(), "按距离排序功能尚未实现");
    }

    return ErrorCode::SUCCESS;
}

}
