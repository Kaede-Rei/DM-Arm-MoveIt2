#ifndef _end_effector_cmd_hpp_
#define _end_effector_cmd_hpp_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief ErrorCode 枚举类：用于表示末端执行器命令执行过程中可能出现的各种错误情况
 * @param SUCCESS 表示操作成功完成
 * @param ASYNC_TASK_RUNNING 表示当前已有异步任务正在执行，无法执行新任务
 * @param INVALID_TARGET_TYPE 表示提供的目标类型无效，无法识别或处理
 * @param TF_TRANSFORM_FAILED 表示坐标变换失败，可能是由于TF树中缺少必要的变换或变换数据不正确导致
 * @param PLANNING_FAILED 表示规划失败，可能是由于环境约束、目标不可达或其他规划问题导致的
 * @param EXECUTION_FAILED 表示执行失败，可能是由于机器人状态不允许执行、执行过程中发生错误或其他执行问题导致的
 * @param TIME_PARAM_FAILED 表示时间参数化失败，可能是由于轨迹不可行、参数化算法失败或其他时间参数化问题导致的
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

/**
 * @brief ArmController 类：用于规划机械臂的运动
 */
class ArmController {
public:
    ArmController(rclcpp::Node::SharedPtr node, const std::string& group_name);
    ~ArmController();

    void home();
    ErrorCode set_joints(const std::vector<double>& joint_values);
    ErrorCode set_target(const TargetVariant& target);
    ErrorCode set_target_in_eef_frame(const TargetVariant& target);
    void clear_target();
    ErrorCode telescopic_end(double length);
    ErrorCode rotate_end(double angle);
    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
    ErrorCode plan_and_execute();
    ErrorCode async_plan_and_execute(std::function<void(ErrorCode)> callback = nullptr);
    void stop();

    ErrorCode parameterize_time(moveit_msgs::msg::RobotTrajectory& trajectory, TimeParamMethod method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult plan_decartes(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult set_line(const TargetVariant& start, const TargetVariant& end, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments = 30, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    ErrorCode execute(const moveit_msgs::msg::RobotTrajectory& trajectory);
    ErrorCode async_execute(const moveit_msgs::msg::RobotTrajectory& trajectory, std::function<void(ErrorCode)> callback = nullptr);

    void set_orientation_constraint(const geometry_msgs::msg::Quaternion& target_orientation, double tolerance_x = 0.1, double tolerance_y = 0.1, double tolerance_z = 0.3, double weight = 1.0);
    void set_position_constraint(const geometry_msgs::msg::Point& target_position, const geometry_msgs::msg::Vector3& scope_size, double weight = 1.0);
    void set_joint_constraint(const std::string& joint_name, double target_angle, double above, double below, double weight = 1.0);
    void apply_constraints();
    void clear_constraints();

    bool is_planning_or_executing() const;
    ErrorCode cancel_async();

    geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
    geometry_msgs::msg::Pose rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z);
    template<class T>
    ErrorCode base_to_end_tf(const T& in, T& out);
    template<class T>
    ErrorCode end_to_base_tf(const T& in, T& out);

    std::vector<double> get_current_joints() const;
    std::vector<std::string> get_current_link_names() const;
    geometry_msgs::msg::Pose get_current_pose() const;

private:
    rclcpp::Node::SharedPtr _node_;
    moveit::planning_interface::MoveGroupInterface _arm_;

    std::unique_ptr<tf2_ros::Buffer> _tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener_;

    const std::string _base_link_;
    const std::string _eef_link_;

    double _vel_scale_;
    double _acc_scale_;
    double _eef_step_;
    double _jump_threshold_;
    double _min_success_rate_;

    moveit_msgs::msg::Constraints _constraints_;

    std::atomic<bool> _is_planning_or_executing_{ false };
    std::thread _async_thread_;
};


// ! ========================= 接 口 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //

/**
 * @brief 将输入的位姿、位置或姿态从底座坐标系转换到末端执行器坐标系
 * @param in 输入的位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion / geometry_msgs::msg::PoseStamped)
 * @param out 转换后的位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion / geometry_msgs::msg::PoseStamped)
 * @return 转换是否成功
 */
template<class T>
ErrorCode ArmController::base_to_end_tf(const T& in, T& out) {
    // 检查输入类型
    static_assert(
        std::is_same_v<T, geometry_msgs::msg::Pose> ||
        std::is_same_v<T, geometry_msgs::msg::Point> ||
        std::is_same_v<T, geometry_msgs::msg::Quaternion> ||
        std::is_same_v<T, geometry_msgs::msg::PoseStamped>,
        "仅支持 Pose(Stamped)、Point 和 Quaternion 的坐标变换");

    try {
        // 带时间戳则直接变换
        if constexpr(std::is_same_v<T, geometry_msgs::msg::PoseStamped>) {
            auto tf_stamped = _tf_buffer_->lookupTransform(_eef_link_, _base_link_, in.header.stamp);
            tf2::doTransform(in, out, tf_stamped);
        }
        // 不带时间戳则构造 PoseStamped 进行变换后提取
        else {
            geometry_msgs::msg::PoseStamped pose_in;
            pose_in.header.frame_id = _base_link_;
            pose_in.header.stamp = _node_->now();

            // 构造 PoseStamped
            if constexpr(std::is_same_v<T, geometry_msgs::msg::Pose>) {
                pose_in.pose = in;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Point>) {
                pose_in.pose.position = in;
                pose_in.pose.orientation.w = 1.0;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Quaternion>) {
                pose_in.pose.orientation = in;
            }

            // 进行坐标变换
            geometry_msgs::msg::PoseStamped pose_out = _tf_buffer_->transform(pose_in, _eef_link_);
            if constexpr(std::is_same_v<T, geometry_msgs::msg::Pose>) {
                out = pose_out.pose;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Point>) {
                out = pose_out.pose.position;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Quaternion>) {
                out = pose_out.pose.orientation;
            }
        }
        RCLCPP_INFO(_node_->get_logger(), "坐标变换成功：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        RCLCPP_WARN(_node_->get_logger(), "坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

/**
 * @brief 将输入的位姿、位置或姿态从末端执行器坐标系转换到底座坐标系
 * @param in 输入的位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion / geometry_msgs::msg::PoseStamped)
 * @param out 转换后的位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion / geometry_msgs::msg::PoseStamped)
 * @return 转换是否成功
 */
template<class T>
ErrorCode ArmController::end_to_base_tf(const T& in, T& out) {
    // 检查输入类型
    static_assert(
        std::is_same_v<T, geometry_msgs::msg::Pose> ||
        std::is_same_v<T, geometry_msgs::msg::Point> ||
        std::is_same_v<T, geometry_msgs::msg::Quaternion> ||
        std::is_same_v<T, geometry_msgs::msg::PoseStamped>,
        "仅支持 Pose(Stamped)、Point 和 Quaternion 的坐标变换");

    try {
        // 带时间戳则直接变换
        if constexpr(std::is_same_v<T, geometry_msgs::msg::PoseStamped>) {
            auto tf_stamped = _tf_buffer_->lookupTransform(_base_link_, _eef_link_, in.header.stamp);
            tf2::doTransform(in, out, tf_stamped);
        }
        // 不带时间戳则构造 PoseStamped 进行变换后提取
        else {
            geometry_msgs::msg::PoseStamped pose_in;
            pose_in.header.frame_id = _eef_link_;
            pose_in.header.stamp = _node_->now();

            // 构造 PoseStamped
            if constexpr(std::is_same_v<T, geometry_msgs::msg::Pose>) {
                pose_in.pose = in;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Point>) {
                pose_in.pose.position = in;
                pose_in.pose.orientation.w = 1.0;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Quaternion>) {
                pose_in.pose.orientation = in;
            }

            // 进行坐标变换
            geometry_msgs::msg::PoseStamped pose_out = _tf_buffer_->transform(pose_in, _base_link_);
            if constexpr(std::is_same_v<T, geometry_msgs::msg::Pose>) {
                out = pose_out.pose;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Point>) {
                out = pose_out.pose.position;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::msg::Quaternion>) {
                out = pose_out.pose.orientation;
            }
        }
        RCLCPP_INFO(_node_->get_logger(), "坐标变换成功：%s -> %s", _eef_link_.c_str(), _base_link_.c_str());
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        RCLCPP_WARN(_node_->get_logger(), "坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

} /* namespace dm_arm */

#endif
