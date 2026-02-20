#ifndef _end_effector_cmd_hpp_
#define _end_effector_cmd_hpp_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

using TargetVariant = std::variant<
    geometry_msgs::msg::Pose,
    geometry_msgs::msg::Point,
    geometry_msgs::msg::Quaternion
>;

/**
 * @brief EndEffectorCmd 类：提供一个接口来设置机械臂末端执行器的目标位姿、位置或姿态
 */
class EndEffectorCmd {
public:
    EndEffectorCmd(rclcpp::Node::SharedPtr node, const std::string& group_name);
    ~EndEffectorCmd() = default;

    bool set_joints(const std::vector<double>& joint_values);
    bool set_target(const TargetVariant& target);
    bool set_target_on_end(const TargetVariant& target);
    bool plan();
    bool plan_and_execute();

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
