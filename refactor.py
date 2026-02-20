import re

def process_hpp():
    with open('src/dm_arm_controller/include/dm_arm_controller/end_effector_cmd.hpp', 'r') as f:
        hpp = f.read()

    enum_str = """
// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

enum class ErrorCode {
    SUCCESS = 0,
    ASYNC_TASK_RUNNING,
    INVALID_TARGET_TYPE,
    TF_TRANSFORM_FAILED,
    PLANNING_FAILED,
    EXECUTION_FAILED,
    TIME_PARAM_FAILED,
    EMPTY_WAYPOINTS,
    DESCARTES_PLANNING_FAILED,
    TARGET_OUT_OF_BOUNDS
};

std::string to_string(ErrorCode code);
"""
    hpp = hpp.replace('// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //\n', enum_str)

    hpp = re.sub(r'bool success;\n\s*double success_rate;', 'ErrorCode error_code;\n    double success_rate;', hpp)

    replacements = [
        (r'bool set_joints', 'ErrorCode set_joints'),
        (r'bool set_target\(', 'ErrorCode set_target('),
        (r'bool set_target_on_end', 'ErrorCode set_target_on_end'),
        (r'bool telescopic_end', 'ErrorCode telescopic_end'),
        (r'bool rotate_end', 'ErrorCode rotate_end'),
        (r'bool plan\(', 'ErrorCode plan('),
        (r'bool plan_and_execute', 'ErrorCode plan_and_execute'),
        (r'bool async_plan_and_execute\(std::function<void\(bool\)>', 'ErrorCode async_plan_and_execute(std::function<void(ErrorCode)>'),
        (r'bool parameterize_time', 'ErrorCode parameterize_time'),
        (r'bool execute\(', 'ErrorCode execute('),
        (r'bool async_execute\(const moveit_msgs::msg::RobotTrajectory& trajectory, std::function<void\(bool\)>', 'ErrorCode async_execute(const moveit_msgs::msg::RobotTrajectory& trajectory, std::function<void(ErrorCode)>'),
        (r'bool cancel_async', 'ErrorCode cancel_async'),
        (r'bool base_to_end_tf', 'ErrorCode base_to_end_tf'),
        (r'bool end_to_base_tf', 'ErrorCode end_to_base_tf')
    ]
    for old, new in replacements:
        hpp = re.sub(old, new, hpp)

    # Fix template returns
    hpp = hpp.replace('RCLCPP_ERROR(_node_->get_logger(), "坐标变换失败：%s", e.what());\n        return false;', 'RCLCPP_WARN(_node_->get_logger(), "坐标变换失败：%s", e.what());\n        return ErrorCode::TF_TRANSFORM_FAILED;')
    hpp = re.sub(r'return true;(\s*\}\n\s*catch)', r'return ErrorCode::SUCCESS;\1', hpp)

    with open('src/dm_arm_controller/include/dm_arm_controller/end_effector_cmd.hpp', 'w') as f:
        f.write(hpp)

def process_cpp():
    with open('src/dm_arm_controller/src/end_effector_cmd.cpp', 'r') as f:
        cpp = f.read()

    to_string_impl = """namespace dm_arm {

std::string to_string(ErrorCode code) {
    switch(code) {
        case ErrorCode::SUCCESS: return "SUCCESS";
        case ErrorCode::ASYNC_TASK_RUNNING: return "ASYNC_TASK_RUNNING";
        case ErrorCode::INVALID_TARGET_TYPE: return "INVALID_TARGET_TYPE";
        case ErrorCode::TF_TRANSFORM_FAILED: return "TF_TRANSFORM_FAILED";
        case ErrorCode::PLANNING_FAILED: return "PLANNING_FAILED";
        case ErrorCode::EXECUTION_FAILED: return "EXECUTION_FAILED";
        case ErrorCode::TIME_PARAM_FAILED: return "TIME_PARAM_FAILED";
        case ErrorCode::EMPTY_WAYPOINTS: return "EMPTY_WAYPOINTS";
        case ErrorCode::DESCARTES_PLANNING_FAILED: return "DESCARTES_PLANNING_FAILED";
        case ErrorCode::TARGET_OUT_OF_BOUNDS: return "TARGET_OUT_OF_BOUNDS";
        default: return "UNKNOWN_ERROR";
    }
}
"""
    cpp = cpp.replace('namespace dm_arm {\n', to_string_impl)

    # Replace all RCLCPP_ERROR with RCLCPP_WARN
    cpp = cpp.replace('RCLCPP_ERROR', 'RCLCPP_WARN')

    # Update method signatures
    replacements = [
        (r'bool EndEffectorCmd::set_joints', 'ErrorCode EndEffectorCmd::set_joints'),
        (r'bool EndEffectorCmd::set_target\(', 'ErrorCode EndEffectorCmd::set_target('),
        (r'bool EndEffectorCmd::set_target_on_end', 'ErrorCode EndEffectorCmd::set_target_on_end'),
        (r'bool EndEffectorCmd::telescopic_end', 'ErrorCode EndEffectorCmd::telescopic_end'),
        (r'bool EndEffectorCmd::rotate_end', 'ErrorCode EndEffectorCmd::rotate_end'),
        (r'bool EndEffectorCmd::plan\(', 'ErrorCode EndEffectorCmd::plan('),
        (r'bool EndEffectorCmd::plan_and_execute', 'ErrorCode EndEffectorCmd::plan_and_execute'),
        (r'bool EndEffectorCmd::async_plan_and_execute\(std::function<void\(bool\)>', 'ErrorCode EndEffectorCmd::async_plan_and_execute(std::function<void(ErrorCode)>'),
        (r'bool EndEffectorCmd::parameterize_time', 'ErrorCode EndEffectorCmd::parameterize_time'),
        (r'bool EndEffectorCmd::execute\(', 'ErrorCode EndEffectorCmd::execute('),
        (r'bool EndEffectorCmd::async_execute\(const moveit_msgs::msg::RobotTrajectory& trajectory, std::function<void\(bool\)>', 'ErrorCode EndEffectorCmd::async_execute(const moveit_msgs::msg::RobotTrajectory& trajectory, std::function<void(ErrorCode)>'),
        (r'bool EndEffectorCmd::cancel_async', 'ErrorCode EndEffectorCmd::cancel_async')
    ]
    for old, new in replacements:
        cpp = re.sub(old, new, cpp)

    # Fix returns in set_joints
    cpp = cpp.replace('return false;\n    }\n    return _arm_.setJointValueTarget(joint_values);', 'return ErrorCode::ASYNC_TASK_RUNNING;\n    }\n    bool success = _arm_.setJointValueTarget(joint_values);\n    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;')

    # Fix returns in set_target
    cpp = cpp.replace('return false;\n    }\n\n    bool success = false;', 'return ErrorCode::ASYNC_TASK_RUNNING;\n    }\n\n    bool success = false;')
    cpp = cpp.replace('return false;\n    }\n\n    return success;', 'return ErrorCode::INVALID_TARGET_TYPE;\n    }\n\n    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;')

    # Fix returns in set_target_on_end
    cpp = cpp.replace('return false;\n    }\n\n    return success;\n}', 'return ErrorCode::INVALID_TARGET_TYPE;\n    }\n\n    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;\n}')

    # Fix returns in plan
    cpp = cpp.replace('return false;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "正在规划...");', 'return ErrorCode::ASYNC_TASK_RUNNING;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "正在规划...");')
    cpp = cpp.replace('return false;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "规划成功");\n    return true;', 'return ErrorCode::PLANNING_FAILED;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "规划成功");\n    return ErrorCode::SUCCESS;')

    # Fix returns in plan_and_execute
    cpp = cpp.replace('return false;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "规划成功，正在执行...");', 'return ErrorCode::PLANNING_FAILED;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "规划成功，正在执行...");')
    cpp = cpp.replace('return false;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "执行成功");\n    return true;', 'return ErrorCode::EXECUTION_FAILED;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "执行成功");\n    return ErrorCode::SUCCESS;')

    # Fix returns in async_plan_and_execute
    cpp = cpp.replace('if(callback) callback(false);', 'if(callback) callback(ErrorCode::PLANNING_FAILED);')
    cpp = cpp.replace('if(callback) callback(ErrorCode::PLANNING_FAILED);\n            }\n            else {\n                RCLCPP_INFO(_node_->get_logger(), "异步执行成功");\n                _is_planning_or_executing_ = false;\n                if(callback) callback(true);', 'if(callback) callback(ErrorCode::EXECUTION_FAILED);\n            }\n            else {\n                RCLCPP_INFO(_node_->get_logger(), "异步执行成功");\n                _is_planning_or_executing_ = false;\n                if(callback) callback(ErrorCode::SUCCESS);')
    cpp = cpp.replace('return false;\n    }\n\n    if(_async_thread_.joinable())', 'return ErrorCode::ASYNC_TASK_RUNNING;\n    }\n\n    if(_async_thread_.joinable())')
    cpp = cpp.replace('});\n\n    return true;', '});\n\n    return ErrorCode::SUCCESS;')

    # Fix returns in parameterize_time
    cpp = cpp.replace('return false;\n    }\n\n    if(!time_param_success)', 'return ErrorCode::TIME_PARAM_FAILED;\n    }\n\n    if(!time_param_success)')
    cpp = cpp.replace('return false;\n    }\n\n    // 将时间参数化后的轨迹转换回消息格式\n    rt.getRobotTrajectoryMsg(trajectory);\n    return true;', 'return ErrorCode::TIME_PARAM_FAILED;\n    }\n\n    // 将时间参数化后的轨迹转换回消息格式\n    rt.getRobotTrajectoryMsg(trajectory);\n    return ErrorCode::SUCCESS;')

    # Fix returns in plan_decartes
    cpp = cpp.replace('result.success = false;', 'result.error_code = ErrorCode::SUCCESS;')
    cpp = cpp.replace('result.error_code = ErrorCode::SUCCESS;\n\n    if (_is_planning_or_executing_) {\n        result.message = "当前已有异步任务正在执行，无法进行新的笛卡尔规划";\n        return result;', 'result.error_code = ErrorCode::ASYNC_TASK_RUNNING;\n\n    if (_is_planning_or_executing_) {\n        result.message = "当前已有异步任务正在执行，无法进行新的笛卡尔规划";\n        return result;')
    cpp = cpp.replace('if(waypoints.empty()) {\n        result.message = "路径点列表为空";\n        return result;\n    }', 'if(waypoints.empty()) {\n        result.error_code = ErrorCode::EMPTY_WAYPOINTS;\n        result.message = "路径点列表为空";\n        return result;\n    }')
    cpp = cpp.replace('if(success_rate <= 0.0) {\n        result.message = "笛卡尔路径规划失败，无法生成有效的轨迹";\n        return result;\n    }', 'if(success_rate <= 0.0) {\n        result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;\n        result.message = "笛卡尔路径规划失败，无法生成有效的轨迹";\n        return result;\n    }')
    cpp = cpp.replace('if(!parameterize_time(trajectory, time_param_method, vel_scale, acc_scale)) {\n        result.message = "时间参数化失败";\n        return result;\n    }', 'if(parameterize_time(trajectory, time_param_method, vel_scale, acc_scale) != ErrorCode::SUCCESS) {\n        result.error_code = ErrorCode::TIME_PARAM_FAILED;\n        result.message = "时间参数化失败";\n        return result;\n    }')
    cpp = cpp.replace('result.success = false;\n        std::stringstream ss;\n        ss << "笛卡尔路径规划失败，成功率：" << (success_rate * 100.0) << "%";', 'result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;\n        std::stringstream ss;\n        ss << "笛卡尔路径规划失败，成功率：" << (success_rate * 100.0) << "%";')
    cpp = cpp.replace('result.success = true;\n        std::stringstream ss;\n        ss << "笛卡尔路径规划成功，成功率：" << (success_rate * 100.0) << "%";', 'result.error_code = ErrorCode::SUCCESS;\n        std::stringstream ss;\n        ss << "笛卡尔路径规划成功，成功率：" << (success_rate * 100.0) << "%";')

    # Fix returns in set_line
    cpp = cpp.replace('result.success = false;\n        result.message = "无效的起点类型', 'result.error_code = ErrorCode::INVALID_TARGET_TYPE;\n        result.message = "无效的起点类型')
    cpp = cpp.replace('result.success = false;\n        result.message = "无效的终点类型', 'result.error_code = ErrorCode::INVALID_TARGET_TYPE;\n        result.message = "无效的终点类型')

    # Fix returns in set_bezier_curve
    cpp = cpp.replace('result.success = false;\n        result.message = "无效的途经点类型', 'result.error_code = ErrorCode::INVALID_TARGET_TYPE;\n        result.message = "无效的途经点类型')

    # Fix returns in execute
    cpp = cpp.replace('return false;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "执行成功");\n    return true;', 'return ErrorCode::EXECUTION_FAILED;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "执行成功");\n    return ErrorCode::SUCCESS;')

    # Fix returns in async_execute
    cpp = cpp.replace('if(callback) callback(false);', 'if(callback) callback(ErrorCode::EXECUTION_FAILED);')
    cpp = cpp.replace('if(callback) callback(true);', 'if(callback) callback(ErrorCode::SUCCESS);')

    # Fix returns in cancel_async
    cpp = cpp.replace('return true;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "正在取消异步任务...");', 'return ErrorCode::SUCCESS;\n    }\n\n    RCLCPP_INFO(_node_->get_logger(), "正在取消异步任务...");')
    cpp = cpp.replace('return true;\n}', 'return ErrorCode::SUCCESS;\n}')

    # Fix main function
    cpp = cpp.replace('if(result.success) {', 'if(result.error_code == dm_arm::ErrorCode::SUCCESS) {')

    with open('src/dm_arm_controller/src/end_effector_cmd.cpp', 'w') as f:
        f.write(cpp)

process_hpp()
process_cpp()
