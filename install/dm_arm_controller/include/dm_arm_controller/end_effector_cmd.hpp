#ifndef _end_effector_cmd_hpp_
#define _end_effector_cmd_hpp_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

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
    EndEffectorCmd(rclcpp::Node::SharedPtr& node, const std::string& group_name);
    ~EndEffectorCmd() = default;

    bool set_joints(const std::vector<double>& joint_values);
    geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
    geometry_msgs::msg::Pose rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z);
    bool set_end(const TargetVariant& target);
    bool plan_and_execute();

    std::vector<double> get_current_joints() const;
    std::vector<std::string> get_current_link_names() const;
    geometry_msgs::msg::Pose get_current_pose() const;

private:
    rclcpp::Node::SharedPtr _node_;
    moveit::planning_interface::MoveGroupInterface _arm_;
};


// ! ========================= 接 口 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //




} /* namespace dm_arm */

#endif
