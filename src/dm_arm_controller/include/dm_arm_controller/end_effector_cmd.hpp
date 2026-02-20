#ifndef _end_effector_cmd_hpp_
#define _end_effector_cmd_hpp_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

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
    bool success;
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
 * @brief EndEffectorCmd 类：用于规划末端执行器的运动
 */
class EndEffectorCmd {
public:
    EndEffectorCmd(rclcpp::Node::SharedPtr node, const std::string& group_name);
    ~EndEffectorCmd() = default;

    void home();
    bool set_joints(const std::vector<double>& joint_values);
    bool set_target(const TargetVariant& target);
    bool set_target_on_end(const TargetVariant& target);
    void clear_target();
    bool telescopic_end(double length);
    bool rotate_end(double angle);
    bool plan();
    bool plan_and_execute();
    void stop();

    bool parameterize_time(moveit_msgs::msg::RobotTrajectory& trajectory, TimeParamMethod method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult plan_decartes(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult set_line(const TargetVariant& start, const TargetVariant& end, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    DescartesResult set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments = 30, double eef_step = 0.01, double jump_threshold = 0.0, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    bool execute(const moveit_msgs::msg::RobotTrajectory& trajectory);

    void set_orientation_constraint(const geometry_msgs::msg::Quaternion& target_orientation, double tolerance_x = 0.1, double tolerance_y = 0.1, double tolerance_z = 0.3, double weight = 1.0);
    void set_position_constraint(const geometry_msgs::msg::Point& target_position, const geometry_msgs::msg::Vector3& scope_size, double weight = 1.0);
    void set_joint_constraint(const std::string& joint_name, double target_angle, double upper, double lower, double weight = 1.0);
    void apply_constraints();
    void clear_constraints();

    geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
    geometry_msgs::msg::Pose rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z);
    template<class T>
    bool base_to_end_tf(const T& in, T& out);
    template<class T>
    bool end_to_base_tf(const T& in, T& out);

    std::vector<double> get_current_joints() const;
    std::vector<std::string> get_current_link_names() const;
    geometry_msgs::msg::Pose get_current_pose() const;

private:
    rclcpp::Node::SharedPtr _node_;
    moveit::planning_interface::MoveGroupInterface _arm_;

    std::unique_ptr<tf2_ros::Buffer> _tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> _tf_listener_;

    const std::string& _base_link_;
    const std::string& _eef_link_;

    double _vel_scale_;
    double _acc_scale_;
    double _eef_step_;
    double _jump_threshold_;
    double _min_success_rate_;

    moveit_msgs::msg::Constraints _constraints_;
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
bool EndEffectorCmd::base_to_end_tf(const T& in, T& out) {
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
        return true;
    }
    catch(const tf2::TransformException& e) {
        RCLCPP_ERROR(_node_->get_logger(), "坐标变换失败：%s", e.what());
        return false;
    }
}

/**
 * @brief 将输入的位姿、位置或姿态从末端执行器坐标系转换到底座坐标系
 * @param in 输入的位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion / geometry_msgs::msg::PoseStamped)
 * @param out 转换后的位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion / geometry_msgs::msg::PoseStamped)
 * @return 转换是否成功
 */
template<class T>
bool EndEffectorCmd::end_to_base_tf(const T& in, T& out) {
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
        return true;
    }
    catch(const tf2::TransformException& e) {
        RCLCPP_ERROR(_node_->get_logger(), "坐标变换失败：%s", e.what());
        return false;
    }
}

} /* namespace dm_arm */

#endif
