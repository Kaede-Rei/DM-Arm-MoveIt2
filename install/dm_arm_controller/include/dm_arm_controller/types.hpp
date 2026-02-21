#ifndef _types_hpp_
#define _types_hpp_

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief ErrorCode 枚举类：用于表示机械臂和末端执行器命令执行过程中可能出现的各种错误情况
 * @param SUCCESS 表示操作成功完成
 * @param ASYNC_TASK_RUNNING 表示当前已有异步任务正在执行，无法执行新任务
 * @param INVALID_TARGET_TYPE 表示提供的目标类型无效，无法识别或处理
 * @param TF_TRANSFORM_FAILED 表示坐标变换失败，可能是由于TF树中缺少必要的变换或变换数据不正确导致
 * @param PLANNING_FAILED 表示规划失败，可能是由于环境约束、目标不可达或其他规划问题导致
 * @param EXECUTION_FAILED 表示执行失败，可能是由于机器人状态不允许执行、执行过程中发生错误或其他执行问题导致
 * @param TIME_PARAM_FAILED 表示时间参数化失败，可能是由于轨迹不可行、参数化算法失败或其他时间参数化问题导致
 * @param EMPTY_WAYPOINTS 表示提供的路径点列表为空，无法进行规划
 * @param DESCARTES_PLANNING_FAILED 表示笛卡尔空间规划失败，可能是由于路径点不可达、规划算法失败或其他笛卡尔规划问题导致
 * @param TARGET_OUT_OF_BOUNDS 表示目标超出机器人工作空间或关节限制，无法执行
 */
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

/**
 * @brief err_to_string 函数：将 ErrorCode 枚举值转换为对应的字符串表示，便于日志记录和调试
 * @param code 要转换的 ErrorCode 枚举值
 * @return 对应的字符串表示，例如 "SUCCESS"、"PLANNING_FAILED"
 */
std::string err_to_string(ErrorCode code);

/**
 * @brief 目标类型：位姿（Pose）、位置（Point）、姿态（Quaternion）或带时间戳的位姿（PoseStamped）
 * @param Pose 包含位置和姿态信息，适用于需要同时设置位置和姿态的场景
 * @param Point 仅包含位置，适用于只需要设置位置的场景，姿态保持不变
 * @param Quaternion 仅包含姿态，适用于只需要设置姿态的场景，位置保持不变
 * @param PoseStamped 可用于带时间戳的坐标变换，适用于需要考虑时间因素的场景
 */
using TargetVariant = std::variant<
    geometry_msgs::msg::Pose,
    geometry_msgs::msg::Point,
    geometry_msgs::msg::Quaternion,
    geometry_msgs::msg::PoseStamped
>;

/**
 * @brief DescartesResult 结构体：用于封装笛卡尔空间规划的结果，包括成功与否、成功率、消息和规划得到的轨迹
 * @param success 笛卡尔空间规划是否成功
 * @param success_rate 笛卡尔空间规划的成功率，范围为0.0到1.0
 * @param message 笛卡尔空间规划的相关消息，例如错误信息或成功提示
 * @param trajectory 笛卡尔空间规划得到的轨迹，包含关节空间和笛卡尔空间的轨迹信息
 */
typedef struct {
    ErrorCode error_code;
    double success_rate;
    std::string message;
    moveit_msgs::msg::RobotTrajectory trajectory;
} DescartesResult;

/**
 * @brief TimeParamMethod 枚举类：用于指定时间参数化的方法，包括 TOTG（Time-Optimal Trajectory Generation）和 SPLINE（基于样条插值的时间参数化）
 * @param TOTG 使用时间最优轨迹生成算法进行时间参数化，适用于需要快速执行的场景
 * @param ISP 使用基于样条插值的时间参数化方法，适用于需要平滑运动的场景，可能会牺牲一些执行速度以获得更平滑的轨迹
 */
enum class TimeParamMethod {
    TOTG,
    ISP,
};

} /* namespace dm_arm */

#endif
