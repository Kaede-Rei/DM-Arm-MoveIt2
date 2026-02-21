#ifndef _eef_controller_hpp_
#define _eef_controller_hpp_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "dm_arm_controller/types.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief JointEefInterface 类：关节类末端执行器抽象接口
 */
class JointEefInterface {
public:
    JointEefInterface() = default;
    virtual ~JointEefInterface() = default;

    // 禁止拷贝构造和拷贝赋值
    JointEefInterface(const JointEefInterface&) = delete;
    JointEefInterface& operator=(const JointEefInterface&) = delete;
    // 禁止移动构造和移动赋值
    JointEefInterface(JointEefInterface&&) = delete;
    JointEefInterface& operator=(JointEefInterface&&) = delete;

    /**
     * @brief 获取末端执行器的规划组名称，即 urdf-moveit 中定义的规划组
     * @return 规划组名称字符串引用
     */
    virtual const std::string& get_group_name() const = 0;
    /**
     * @brief 获取末端执行器的 MoveGroupInterface 对象，用于进行运动规划和控制
     * @return MoveGroupInterface 对象的引用，子类需要实现该方法返回对应的 MoveGroupInterface 实例
     */
    virtual moveit::planning_interface::MoveGroupInterface& get_move_group() = 0;

    /**
     * @brief 打开末端执行器控制器
     * @return 打开结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode open() = 0;
    /**
     * @brief 关闭末端执行器控制器
     * @return 关闭结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode close() = 0;

    /**
     * @brief 执行末端执行器的预设位姿，预设位姿是指机器人末端执行器在特定任务或状态下的标准位姿
     * @param pose_name 预设位姿的名称字符串，具体的预设位姿定义由子类实现
     * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode execute_preset_pose(const std::string& pose_name) = 0;
    /**
     * @brief 设置末端执行器的单个关节值
     * @param joint_name 关节名称字符串，具体的关节名称由子类实现
     * @param value 关节的目标位置或角度值，单位和范围由子类实现
     * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode set_joint_value(const std::string& joint_name, double value) = 0;
    /**
     * @brief 设置末端执行器的所有关节值
     * @param joint_values 关节名称和对应目标位置或角度值的映射，单位和范围由子类实现
     * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode set_joint_values(const std::vector<double>& joint_values) = 0;
    /**
     * @brief 规划末端执行器的运动轨迹，生成从当前状态到目标状态的运动路径
     * @param plan 规划结果的输出参数，包含规划生成的轨迹和相关信息
     * @return 规划结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;
    /**
     * @brief 执行末端执行器的运动轨迹，控制机器人按照规划生成的轨迹进行运动
     * @param plan 需要执行的运动轨迹，包含规划生成的轨迹和相关信息
     * @return 执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;
    /**
     * @brief 规划并执行末端执行器设置好的目标状态，包含从当前状态到目标状态的规划和执行过程
     * @param plan 规划结果的输出参数，包含规划生成的轨迹和相关信息
     * @return 规划和执行结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode plan_and_execute() = 0;

    /**
     * @brief 获取末端执行器当前的关节值，返回一个包含所有关节当前值的向量，顺序与规划组中定义的关节顺序一致
     * @return 包含末端执行器当前关节值的向量，单位和范围由子类实现
     */
    virtual std::vector<double> get_current_joints() const = 0;
    /**
     * @brief 获取末端执行器当前的链接名称列表，返回一个包含所有当前链接名称的向量
     * @return 包含末端执行器当前链接名称的向量，每个元素是一个链接名称字符串，具体的链接名称由子类实现
     */
    virtual std::vector<std::string> get_current_link_names() const = 0;
};

/**
 * @brief IoEefInterface 类：IO类末端执行器抽象接口
 */
class IoEefInterface {
public:
    IoEefInterface() = default;
    virtual ~IoEefInterface() = default;

    // 禁止拷贝构造和拷贝赋值
    IoEefInterface(const IoEefInterface&) = delete;
    IoEefInterface& operator=(const IoEefInterface&) = delete;
    // 禁止移动构造和移动赋值
    IoEefInterface(IoEefInterface&&) = delete;
    IoEefInterface& operator=(IoEefInterface&&) = delete;

    /**
     * @brief 获取末端执行器支持的IO名称列表，子类需要实现该方法返回具体的IO名称列表
     * @return IO名称列表，包含末端执行器支持的所有IO的名称字符串
     */
    virtual std::vector<std::string> get_io_names() const = 0;

    /**
     * @brief 启用指定名称的IO，控制末端执行器的特定功能或状态
     * @param io_name 需要启用的IO名称字符串，必须是get_io_names()方法返回的列表中的名称
     * @return 启用结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode enable_io(const std::string& io_name) = 0;
    /**
     * @brief 禁用指定名称的IO，控制末端执行器的特定功能或状态
     * @param io_name 需要禁用的IO名称字符串，必须是get_io_names()方法返回的列表中的名称
     * @return 禁用结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode disable_io(const std::string& io_name) = 0;
    /**
     * @brief 启用所有IO，控制末端执行器的所有功能或状态
     * @return 启用结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode enable_all() = 0;
    /**
     * @brief 禁用所有IO，控制末端执行器的所有功能或状态
     * @return 禁用结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode disable_all() = 0;

};

class PwmEefInterface {
public:
    PwmEefInterface() = default;
    virtual ~PwmEefInterface() = default;

    // 禁止拷贝构造和拷贝赋值
    PwmEefInterface(const PwmEefInterface&) = delete;
    PwmEefInterface& operator=(const PwmEefInterface&) = delete;
    // 禁止移动构造和移动赋值
    PwmEefInterface(PwmEefInterface&&) = delete;
    PwmEefInterface& operator=(PwmEefInterface&&) = delete;

    /**
     * @brief 获取末端执行器支持的PWM名称列表，子类需要实现该方法返回具体的PWM名称列表
     * @return PWM名称列表，包含末端执行器支持的所有PWM的名称
     */
    virtual std::vector<std::string> get_io_names() const = 0;
    /**
     * @brief 设置指定io的PWM值
     * @param io_name 需要设置的PWM名称字符串，必须是get_io_names()方法返回的列表中的名称
     * @param pwm_value 需要设置的PWM值，具体的数值范围和单位由子类实现
     * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode set_pwm(const std::string& io_name, double pwm_value) = 0;
    /**
    * @brief 设置所有io的PWM值
    * @param pwm_value 需要设置的PWM值，具体的数值范围和单位由子类实现
    * @return 设置结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
    */
    virtual ErrorCode set_all_pwm(double pwm_value) = 0;
    /**
     * @brief 获取指定io的PWM值
     * @param io_name 需要获取的PWM名称字符串，必须是get_io_names()方法返回的列表中的名称
     * @param pwm_value 获取到的PWM值，具体的数值范围和单位由子类实现
     * @return 获取结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode get_pwm(const std::string& io_name, double& pwm_value) const = 0;
    /**
     * @brief 获取所有io的PWM值
     * @param pwm_values 获取到的PWM值映射，包含所有io名称和对应的PWM值，具体的数值范围和单位由子类实现
     * @return 获取结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode get_all_pwm(std::map<std::string, double>& pwm_values) const = 0;
};

/**
 * @brief ForceFeedbackEefInterface 类：力反馈类末端执行器抽象接口
 */
class ForceFeedbackEefInterface {
public:
    ForceFeedbackEefInterface() = default;
    virtual ~ForceFeedbackEefInterface() = default;

    // 禁止拷贝构造和拷贝赋值
    ForceFeedbackEefInterface(const ForceFeedbackEefInterface&) = delete;
    ForceFeedbackEefInterface& operator=(const ForceFeedbackEefInterface&) = delete;
    // 禁止移动构造和移动赋值
    ForceFeedbackEefInterface(ForceFeedbackEefInterface&&) = delete;
    ForceFeedbackEefInterface& operator=(ForceFeedbackEefInterface&&) = delete;

    /**
     * @brief 获取末端执行器支持的力反馈名称列表，子类需要实现该方法返回具体的力反馈名称列表
     * @return 力反馈名称列表，包含末端执行器支持的所有力反馈的名称字符串
     */
    virtual std::vector<std::string> get_force_names() const = 0;
    /**
     * @brief 获取指定名称的力反馈值，包含力的大小和方向等信息
     * @param force_name 需要获取的力反馈名称字符串，必须是get_force_names()方法返回的列表中的名称
     * @param force_value 获取到的力反馈值，具体的数值范围和单位由子类实现
     * @return 获取结果的错误码，成功返回ErrorCode::SUCCESS，失败返回相应的错误码
     */
    virtual ErrorCode get_force(const std::string& force_name, double& force_value) const = 0;
};

/**
 * @brief EndEffector 类：末端执行器抽象接口类，定义了末端执行器控制器的基本功能和接口
 */
class EndEffector {
public:
    /**
     * @brief 构造函数：初始化末端执行器控制器
     * @param node ROS2节点的共享指针，用于创建发布者、订阅者和服务等ROS2通信机制
     * @param eef_name 末端执行器的名称，用于区分不同的末端执行器，便于日志记录和调试
     */
    explicit EndEffector(rclcpp::Node::SharedPtr node, const std::string& eef_name) : _node_(std::move(node)), _eef_name_(eef_name) {};
    virtual ~EndEffector() = default;

    // 禁止拷贝构造和拷贝赋值
    EndEffector(const EndEffector&) = delete;
    EndEffector& operator=(const EndEffector&) = delete;
    // 禁止移动构造和移动赋值
    EndEffector(EndEffector&&) = delete;
    EndEffector& operator=(EndEffector&&) = delete;

    /**
     * @brief 获取末端执行器的名称
     * @return 末端执行器的名称字符串引用
     */
    virtual const std::string& get_eef_name() const { return _eef_name_; }

    /**
     * @brief 停止末端执行器动作
     */
    virtual void stop() = 0;

    /**
     * @brief 获取末端执行器支持的功能类型，子类可以重写这些方法来指示支持的功能
     */
    virtual bool supports_joint_control() const { return false; }
    /**
     * @brief 获取末端执行器支持的功能类型，子类可以重写这些方法来指示支持的功能
     */
    virtual bool supports_io_control() const { return false; }
    /**
     * @brief 获取末端执行器支持的功能类型，子类可以重写这些方法来指示支持的功能
     */
    virtual bool supports_fluid_control() const { return false; }
    /**
     * @brief 获取末端执行器支持的功能类型，子类可以重写这些方法来指示支持的功能
     */
    virtual bool supports_force_feedback() const { return false; }
    /**
     * @brief 获取末端执行器支持的功能类型，子类可以重写这些方法来指示支持的功能
     */
    virtual bool supports_grasp_planning() const { return false; }

    /**
     * @brief 设置末端执行器的TCP（工具中心点）偏移，定义末端执行器相对于机器人末端的位姿偏移
     */
    void set_tcp_offset(const geometry_msgs::msg::Pose& tcp_offset) { _tcp_offset_ = tcp_offset; }
    /**
     * @brief 获取末端执行器的TCP（工具中心点）偏移，定义末端执行器相对于机器人末端的位姿偏移
     * @return 末端执行器的TCP偏移位姿
     */
    const geometry_msgs::msg::Pose& get_tcp_offset() const { return _tcp_offset_; }

protected:
    rclcpp::Node::SharedPtr node() const { return _node_; }

private:
    rclcpp::Node::SharedPtr _node_;
    std::string _eef_name_;
    geometry_msgs::msg::Pose _tcp_offset_;
};

/**
 * @brief TwoFingerGripper 类：二指夹爪具体实现类
 * @note 该类继承自 EndEffector 类，并实现了 JointEefInterface 和 ForceFeedbackEefInterface 接口
 * @note 提供了二指夹爪的具体控制功能，包括关节控制和力反馈功能
 */
class TwoFingerGripper : public EndEffector,
    public JointEefInterface,
    public ForceFeedbackEefInterface {
public:
    explicit TwoFingerGripper(rclcpp::Node::SharedPtr node,
        const std::string& eef_name);
    ~TwoFingerGripper() override = default;

    void stop() override;

    const std::string& get_group_name() const override;
    moveit::planning_interface::MoveGroupInterface& get_move_group() override;
    ErrorCode open() override;
    ErrorCode close() override;
    ErrorCode execute_preset_pose(const std::string& pose_name) override;
    ErrorCode set_joint_value(const std::string& joint_name, double value) override;
    ErrorCode set_joint_values(const std::vector<double>& joint_values) override;
    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) override;
    ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) override;
    ErrorCode plan_and_execute() override;

    std::vector<double> get_current_joints() const override;
    std::vector<std::string> get_current_link_names() const override;

    bool supports_joint_control() const override { return true; }
    bool supports_force_feedback() const override { return true; }

    std::vector<std::string> get_force_names() const override;
    ErrorCode get_force(const std::string& force_name, double& force_value) const override;

private:
    moveit::planning_interface::MoveGroupInterface _gripper_;
};

// ! ========================= 接 口 函 数 声 明 ========================= ! //



// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
