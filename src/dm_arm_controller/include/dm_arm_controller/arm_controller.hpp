#ifndef _arm_controller_hpp_
#define _arm_controller_hpp_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "dm_arm_controller/types.hpp"

// 前向声明
namespace dm_arm {
class EndEffector;
}

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief ArmController 类：用于规划机械臂的运动
 */
class ArmController {
public:
    explicit ArmController(rclcpp::Node::SharedPtr node, const std::string& group_name);
    ~ArmController();

    // 禁止拷贝构造与拷贝赋值
    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;
    // 禁止移动构造与移动赋值
    ArmController(ArmController&&) = delete;
    ArmController& operator=(ArmController&&) = delete;

    /**
     * @brief 获取末端执行器共享指针
     * @param eef 末端执行器的共享指针
     */
    void attach_eef(std::shared_ptr<EndEffector> eef) { _eef_ = std::move(eef); }

    void home();
    ErrorCode set_joints(const std::vector<double>& joint_values);
    ErrorCode set_target(const TargetVariant& target);
    ErrorCode set_target_in_eef_frame(const TargetVariant& target);
    void clear_target();
    ErrorCode telescopic_end(double length);
    ErrorCode rotate_end(double angle);
    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
    ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
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
    /// @brief 节点指针
    rclcpp::Node::SharedPtr _node_;
    /// @brief MoveGroupInterface 对象
    moveit::planning_interface::MoveGroupInterface _arm_;
    /// @brief 末端执行器对象的共享指针
    std::shared_ptr<EndEffector> _eef_;

    /// @brief TF2 缓冲区
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer_;
    /// @brief TF2 监听器
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener_;

    /// @brief 机械臂底座坐标系名称
    const std::string _base_link_;
    /// @brief 机械臂末端执行器坐标系名称
    const std::string _eef_link_;

    /// @brief 速度缩放因子
    double _vel_scale_;
    /// @brief 加速度缩放因子
    double _acc_scale_;

    /// @brief 末端执行器线性插补步长
    double _eef_step_;
    /// @brief 跳跃阈值（用于 Descartes 规划）
    double _jump_threshold_;
    /// @brief 最小成功率（用于 Descartes 规划结果评估）
    double _min_success_rate_;

    /// @brief 运动约束
    moveit_msgs::msg::Constraints _constraints_;

    /// @brief 异步规划执行布尔值
    std::atomic<bool> _is_planning_or_executing_{ false };
    /// @brief 异步规划执行实例
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
