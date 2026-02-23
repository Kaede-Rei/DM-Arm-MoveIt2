#ifndef _tasks_manager_hpp_
#define _tasks_manager_hpp_

#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/container.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "dm_arm_controller/types.hpp"
#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_controller/eef_controller.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 任务类型
 * @param NONE 无特定任务
 * @param PICK_AND_PLACE 抓取和放置任务
 * @param MOVE_TO_HOME 移动到初始位任务
 */
enum class TaskType {
    NONE = 0,
    PICK_AND_PLACE,
    MOVE_TO_HOME,
};

/**
 * @brief 任务状态
 * @param PENDING 等待执行
 * @param PLANNING 正在规划
 * @param EXECUTING 正在执行
 * @param SUCCEEDED 执行成功
 * @param FAILED 执行失败
 * @param CANCELLED 已取消
 * @param ROLLED_BACK 已回滚
 */
enum class TaskStatus {
    PENDING = 0,
    PLANNING,
    EXECUTING,
    SUCCEEDED,
    FAILED,
    CANCELLED,
    ROLLED_BACK,
};

/**
 * @brief 任务组执行模式
 * @param BY_ID 按照任务 ID 顺序执行
 * @param BY_SHORTEST_PATH 按最短路径执行
 */
enum class ExecutionMode {
    BY_ID = 0,
    BY_SHORTEST_PATH,
};

/**
 * @brief 任务描述符
 * @param id 任务 ID
 * @param type 任务类型
 * @param status 任务状态
 * @param error_code 错误码
 * @param message 任务相关消息
 * @param approach_distance 接近距离
 * @param retreat_distance 撤退距离
 * @param lift_distance 抬升距离
 * @param object_id 目标物体 ID（Pick and Place 任务）
 * @param grasp_pose 抓取目标位姿（Pick and Place 任务）
 * @param place_pose 放置目标位姿（Pick and Place 任务）
 * @param support_surface 支撑面 ID（Pick and Place 任务）
 */
struct TaskDescriptor {
    int id{};
    TaskType type = TaskType::NONE;
    TaskStatus status = TaskStatus::PENDING;
    ErrorCode error_code = ErrorCode::SUCCESS;
    std::string message;

    // 通用参数
    double approach_distance = 0.1;             // 接近距离
    double retreat_distance = 0.1;              // 撤退距离
    double lift_distance = 0.05;                // 抬升距离

    // Pick and Place 参数
    std::string object_id;                      // 目标物体 ID
    geometry_msgs::msg::Pose grasp_pose;        // 抓取目标位姿
    geometry_msgs::msg::Pose place_pose;        // 放置目标位姿
    std::string support_surface;                // 支撑面 ID
};

/**
 * @brief 任务组描述符
 * @param group_id 任务组 ID
 * @param name 任务组名称
 * @param mode 任务执行模式
 * @param tasks 任务列表
 */
struct TaskGroup {
    int group_id;
    std::string name;
    ExecutionMode mode = ExecutionMode::BY_ID;
    std::vector<TaskDescriptor> tasks;
};

/**
 * @brief 任务执行反馈结构体
 * @param task_id 任务 ID
 * @param status 任务状态
 * @param stage_name 当前执行阶段名称
 * @param message 相关消息
 * @param progress 执行进度（0.0 ~ 1.0）
 */
struct ExecutionFeedback {
    int task_id;
    TaskStatus status = TaskStatus::PENDING;
    std::string stage_name;
    std::string message;
    double progress = 0.0;                      // 0.0 ~ 1.0
};

using FeedbackCallback = std::function<void(const ExecutionFeedback&)>;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief TasksManager 类：基于 MTC 的多任务管理器
 */
class TasksManager {
public:
    TasksManager(rclcpp::Node::SharedPtr node, const std::string& arm_group_name, const std::string& eef_group_name);
    ~TasksManager();

    // 禁止拷贝构造和拷贝赋值
    TasksManager(const TasksManager&) = delete;
    TasksManager& operator=(const TasksManager&) = delete;
    // 禁止移动构造和移动赋值
    TasksManager(TasksManager&&) = delete;
    TasksManager& operator=(TasksManager&&) = delete;

    std::shared_ptr<ArmController> arm() const { return _arm_; }
    std::shared_ptr<TwoFingerGripper> eef() const { return _eef_; }

    int create_task_group(const std::string& name, ExecutionMode mode = ExecutionMode::BY_ID);
    bool remove_task_group(int group_id);
    void clear_task_groups();

    bool add_task(int group_id, const TaskDescriptor& task);
    bool remove_task(int group_id, int task_id);
    const TaskDescriptor* get_task(int group_id, int task_id) const;
    void clear_tasks(int group_id);

    ErrorCode execute_single_task(int group_id, int task_id);
    ErrorCode execute_task_group(int group_id);
    void cancel();
    bool is_busy() const;

    void set_feedback_callback(FeedbackCallback cb);
    void set_max_retries(int retries);
    void set_max_solutions(size_t n);

private:
    rclcpp::Node::SharedPtr _node_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<TwoFingerGripper> _eef_;

    mutable std::mutex _tasks_mutex_;
    std::map<int, TaskGroup> _task_groups_;
    int _next_group_id_ = 1;

    std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner> _sampling_solver_;
    std::shared_ptr<moveit::task_constructor::solvers::CartesianPath> _cartesian_solver_;
    std::shared_ptr<moveit::task_constructor::solvers::JointInterpolationPlanner> _joint_interp_solver_;

    std::atomic<bool> _is_busy_{ false };
    std::atomic<bool> _cancel_requested_{ false };
    FeedbackCallback _feedback_cb_;
    int _max_retries_;
    size_t _max_solutions_;

    std::string _arm_group_name_;
    std::string _eef_group_name_;
    std::string _hand_frame_;

    void publish_feedback(int task_id, TaskStatus status, const std::string& stage, const std::string& msg, double progress = 0.0);
    void update_task_status(int group_id, int task_id, TaskStatus status, ErrorCode ec = ErrorCode::SUCCESS, const std::string& msg = "");

    ErrorCode execute_task_internal(const TaskDescriptor& desc);

    moveit::task_constructor::TaskPtr build_move_to_home_task();
    moveit::task_constructor::TaskPtr build_pick_and_place_task(const TaskDescriptor& desc);

    ErrorCode plan_mtc_task(moveit::task_constructor::Task& task, size_t max_solutions);
    ErrorCode execute_mtc_task(moveit::task_constructor::Task& task);

    std::vector<int> order_by_id(const TaskGroup& group) const;
    std::vector<int> order_by_shortest_path(const TaskGroup& group) const;
    static double pose_distance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b);
    geometry_msgs::msg::Pose get_task_target_pose(const TaskDescriptor& desc) const;

    void rollback(int group_id, const std::vector<int>& executed_ids);
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



} /* namespace dm_arm */

#endif
