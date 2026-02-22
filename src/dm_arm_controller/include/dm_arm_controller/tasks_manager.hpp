#ifndef _tasks_manager_hpp_
#define _tasks_manager_hpp_

#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "dm_arm_controller/types.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //



// ! ========================= 接 口 函 数 声 明 ========================= ! //

class TasksManager {
public:
    TasksManager(rclcpp::Node::SharedPtr node);
    ~TasksManager() = default;

    enum class TaskType {
        NONE = 0,
        PICK_AND_PLACE,
    };

    enum class SortType {
        BY_ID,
        BY_TYPE,
        BY_DIST,
    };

    struct Task {
        int id;
        TaskType type;
        ErrorCode error_code;
        std::string message;
    };

    void add_task(const Task& task);
    void delete_task(int id);
    void clear_task_group();

    void sort_task_group(std::vector<SortType> sort_types);
    void sort_task_group(SortType sort_type);

    Task execute_task(int id);
    std::vector<Task> execute_task_group();

    Task get_task(int id) const;
    std::vector<Task> get_task_group() const;

private:
    rclcpp::Node::SharedPtr _node_;
    std::vector<Task> _task_group_;

};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
