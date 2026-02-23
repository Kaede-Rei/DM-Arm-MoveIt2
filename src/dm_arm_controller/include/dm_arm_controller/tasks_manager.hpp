#ifndef _tasks_manager_hpp_
#define _tasks_manager_hpp_

#include <rclcpp/rclcpp.hpp>

#include "dm_arm_controller/types.hpp"
#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_controller/eef_controller.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

enum class TaskType {
    NONE = 0,
    PICK,
};

enum class SortType {
    ID,
    DIST,
};

struct Task {
    int id;
    std::string desc;
    TaskType type;

    TargetVariant target;
};

struct TaskGroup {
    std::map<int, Task> tasks;
    std::vector<Task> sorted_tasks;

    SortType sort_type;
};

class TasksManager {
public:
    TasksManager(rclcpp::Node::SharedPtr node, std::shared_ptr<ArmController> arm, std::shared_ptr<EndEffector> eef);
    ~TasksManager() = default;

    // 禁止构造复制和构造赋值
    TasksManager(const TasksManager&) = delete;
    TasksManager& operator=(const TasksManager&) = delete;
    // 禁止移动构造和移动赋值
    TasksManager(TasksManager&&) = delete;
    TasksManager& operator=(TasksManager&&) = delete;

    ErrorCode create_task_group(const std::string& group_name, SortType sort_type = SortType::ID);
    ErrorCode delete_task_group(const std::string& group_name);
    ErrorCode clear_task_group(const std::string& group_name);

    ErrorCode add_task(const std::string& group_name, int id, TaskType task_type = TaskType::NONE, const std::string& task_description = "");
    ErrorCode delete_task(const std::string& group_name, int id);
    ErrorCode set_task_target(const std::string& group_name, int id, const TargetVariant& target);

    ErrorCode execute_task(const std::string& group_name, int id);
    ErrorCode execute_task(Task& task);
    ErrorCode execute_task_group(const std::string& group_name);

private:
    rclcpp::Node::SharedPtr _node_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<EndEffector> _eef_;
    std::string _arm_name_;
    std::string _eef_name_;

    std::map<std::string, TaskGroup> _task_groups_;

    Task* find_task(const std::string& group_name, int id, ErrorCode& error_code);
    ErrorCode sort_tasks(TaskGroup& task_group);
};

// ! ========================= 接 口 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
