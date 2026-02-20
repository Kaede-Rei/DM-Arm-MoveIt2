# dm_arm MoveIt2 从零开始完整教程

> 基于 ROS2 Humble + MoveIt2，从最小可用代码逐步扩展至完整系统
> 
> **前置条件：**
> 1. 已完成 `dm_arm_description` 包并在 RViz2 中验证模型显示
> 2. 已使用 MoveIt Setup Assistant 生成 `dm_arm_moveit_config` 包

---

[TOC]

---

## 第一章：MoveIt2 架构深度剖析

### 1.1 整体架构

MoveIt2 的核心设计思想是**分层解耦**，将机器人控制分为三个独立层次：

```
┌─────────────────────────────────────────────────────────────┐
│  应用层 (Your Application)                                   │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │
│  • MoveGroupInterface           (C++ API)                   │
│  • Python API                                               │
│  • dm_arm_server                (本项目服务层封装)             │
└─────────────────────────────────────────────────────────────┘
                             ↓ ↑
                  (Goal / Feedback / Result)
                             ↓ ↑
┌─────────────────────────────────────────────────────────────┐
│  MoveIt2 核心层 (move_group node)                            │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ PlanningRequestAdapter Chain                          │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  FixStartStateBounds → FixWorkspaceBounds →           │  │
│  │  FixStartStateCollision → FixStartStatePathConstraints│  │
│  └───────────────────────────────────────────────────────┘  │
│                          ↓                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ PlanningPipeline                                      │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  • OMPL Planner (RRTConnect/RRTstar/PRM/...)          │  │
│  │  • Pilz Industrial Planner (PTP/LIN/CIRC)             │  │
│  │  • CHOMP / STOMP (轨迹优化)                             │  │
│  └───────────────────────────────────────────────────────┘  │
│                          ↓                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ TrajectoryProcessing                                  │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  IterativeParabolicTimeParameterization               │  │
│  │   (为路径添加速度/加速度/时间戳)                       	   │  │
│  └───────────────────────────────────────────────────────┘  │
│                          ↓                                  │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ TrajectoryExecutionManager                            │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  • 查找控制器 (moveit_controllers.yaml)                 │  │
│  │  • 验证轨迹 (关节限位、速度限制)                           │  │
│  │  • 发送 FollowJointTrajectory Action                   │  │
│  │  • 监控执行状态 (超时/碰撞/抢占)                           │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ PlanningSceneMonitor                                  │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  • 订阅 /joint_states (关节状态)                        │  │
│  │  │  订阅 /planning_scene (场景差量更新)                  │  │
│  │  • 订阅 /collision_object (障碍物)                      │  │
│  │  • 订阅 /attached_collision_object (附着物)             │  │
│  │  • TF 监听器 (坐标系变换)                                │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  → PlanningScene (线程安全的世界模型)                    │  │
│  │     • RobotModel (URDF + SRDF 解析)                   │  │
│  │     • RobotState (当前关节状态 + FK/IK)                 │  │
│  │     • CollisionWorld (障碍物八叉树)                     │  │
│  │     • AllowedCollisionMatrix (ACM)                    │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                             ↓ ↑
              (FollowJointTrajectory Action)
                             ↓ ↑
┌─────────────────────────────────────────────────────────────┐
│  ros2_control 层                                             │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │
│  controller_manager (ROS2 控制节点)                          │
│   ├─ joint_state_broadcaster (发布 /joint_states)           │
│   ├─ arm_controller (JointTrajectoryController)             │
│   │   └─ FollowJointTrajectory Action Server                │
│   └─ gripper_controller (JointTrajectoryController)         │
│                          ↓ ↑                                 │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ HardwareInterface (dm_arm_hardware 插件)              │  │
│  │  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │  │
│  │  export_state_interfaces()                            │  │
│  │   → StateInterface("joint1", "position", &hw_pos[0])  │  │
│  │  export_command_interfaces()                          │  │
│  │   → CommandInterface("joint1", "position", &hw_cmd[0])│  │
│  │                                                        │  │
│  │  read(time, period)   ← 500Hz 循环调用                │  │
│  │   └─ 从电机读取编码器值 → hw_pos[i]                   │  │
│  │  write(time, period)                                  │  │
│  │   └─ 从 hw_cmd[i] 读取目标 → 下发到电机               │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                             ↓ ↑
                         (CAN 总线)
                             ↓ ↑
┌─────────────────────────────────────────────────────────────┐
│  硬件层 (达妙电机 × 7)                                        │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  │
│  joint1-6: 达妙 DM-J4310/DM-J4340 (旋转关节)                 │
│  gripper_left: 达妙 DM-J4310 + 导轨 (直线关节)               │
└─────────────────────────────────────────────────────────────┘
```

---

### 1.2 数据流详解

以 `arm_.setPoseTarget(pose); arm_.move();` 为例，完整数据流：

```
1. MoveGroupInterface::setPoseTarget()
   └─→ 内部存储 geometry_msgs::msg::PoseStamped 到成员变量

2. MoveGroupInterface::move()
   └─→ 调用 plan() + execute()

3. MoveGroupInterface::plan()
   ├─→ 构造 moveit_msgs::srv::GetMotionPlan::Request
   │   ├─ workspace_parameters (工作空间边界)
   │   ├─ start_state (当前关节状态，从 /joint_states 获取)
   │   ├─ goal_constraints (目标约束)
   │   │   └─ PositionConstraint: {x, y, z} ± tolerance
   │   │   └─ OrientationConstraint: {qx, qy, qz, qw} ± tolerance
   │   ├─ path_constraints (路径约束，本例为空)
   │   ├─ planner_id ("RRTConnect")
   │   ├─ group_name ("arm")
   │   ├─ num_planning_attempts (10)
   │   ├─ allowed_planning_time (5.0)
   │   └─ max_velocity_scaling_factor (0.3)
   │
   ├─→ 通过 /plan_kinematic_path 服务发送给 move_group
   │
   ├─→ move_group 收到后进入 PlanningRequestAdapter 链
   │   ├─ FixStartStateBounds: 检查起始关节值是否在限位内，超限则截断
   │   ├─ FixStartStateCollision: 若起点碰撞，微调关节值避开碰撞
   │   └─ FixStartStatePathConstraints: 调整起点满足路径约束
   │
   ├─→ 进入 PlanningPipeline，调用 OMPL RRTConnect
   │   ├─ 设置起点：planning_scene->getCurrentState()
   │   ├─ 设置终点：从 goal_constraints 提取目标位姿
   │   │   └─ 调用 IK 求解器 (kdl_kinematics_plugin)
   │   │       └─ 多次随机采样关节空间种子点，直到找到满足位姿的 IK 解
   │   ├─ 随机采样可行配置（满足关节限位 + 无碰撞）
   │   ├─ 构建 RRT 树，连接起点和终点
   │   │   └─ 每次扩展前调用 planning_scene->isStateValid()
   │   │       ├─ 检查关节限位
   │   │       ├─ 检查自碰撞 (SRDF disable_collisions 豁免)
   │   │       └─ 检查环境碰撞 (CollisionWorld)
   │   └─ 返回路径：std::vector<moveit::core::RobotState>
   │       ├─ 示例：20 个插值点，每个点包含 6 个关节角度
   │       └─ 此时路径**没有时间戳和速度**，仅几何路径
   │
   ├─→ TimeParameterization (时间参数化)
   │   ├─ 输入：关节角度序列 [q0, q1, ..., q20]
   │   ├─ 算法：IterativeParabolicTimeParameterization
   │   │   ├─ 根据 joint_limits.yaml 的 max_velocity 和 max_acceleration
   │   │   ├─ 以及 velocity_scaling_factor=0.3, accel_scaling_factor=0.3
   │   │   └─ 计算每个点的速度、加速度和时间戳
   │   ├─ 输出：trajectory_msgs::msg::JointTrajectory
   │   │   └─ points[i]:
   │   │       ├─ positions: [j1, j2, j3, j4, j5, j6]
   │   │       ├─ velocities: [v1, v2, v3, v4, v5, v6]
   │   │       ├─ accelerations: [a1, a2, a3, a4, a5, a6]
   │   │       └─ time_from_start: Duration(秒)
   │   └─ 示例输出：
   │       points[0]:  positions=[0, 0, 0, 0, 0, 0], time=0.0s
   │       points[5]:  positions=[0.3, 0.5, ...], time=1.2s
   │       points[20]: positions=[1.2, 1.8, ...], time=4.5s (总时长)
   │
   └─→ 返回 moveit::planning_interface::MoveGroupInterface::Plan
       ├─ start_state_: 起始关节状态
       ├─ trajectory_: 完整轨迹（带时间戳）
       └─ planning_time_: 规划耗时（秒）

4. MoveGroupInterface::execute(plan)
   ├─→ TrajectoryExecutionManager::execute()
   │   ├─ 验证轨迹合法性
   │   │   ├─ 检查关节名称匹配
   │   │   ├─ 检查每个点的关节值在限位内
   │   │   └─ 检查速度、加速度不超限
   │   │
   │   ├─ 按关节分组，查找对应控制器
   │   │   └─ moveit_controllers.yaml: arm_controller 管理 [joint1-6]
   │   │
   │   ├─ 构造 control_msgs::action::FollowJointTrajectory::Goal
   │   │   ├─ trajectory: plan.trajectory_.joint_trajectory
   │   │   ├─ goal_time_tolerance: 0.0 (立即开始)
   │   │   └─ path_tolerance / goal_tolerance (从 ros2_controllers.yaml)
   │   │
   │   ├─→ 发送 Action Goal 到 /arm_controller/follow_joint_trajectory
   │   │
   │   ├─→ arm_controller (JointTrajectoryController) 收到目标
   │   │   ├─ 验证轨迹起点与当前状态一致（误差 < 0.01 rad）
   │   │   ├─ 启动轨迹跟踪：每个控制周期（500Hz = 2ms）
   │   │   │   ├─ 根据当前时间插值轨迹点
   │   │   │   │   └─ t=1.234s → 在 points[5] 和 points[6] 之间线性插值
   │   │   │   ├─ 将插值结果写入 hw_commands_[0..5]
   │   │   │   │   └─ hw_commands_[0] = 0.85 (joint1 目标位置)
   │   │   │   └─ 触发 write() 下发到硬件接口
   │   │   │
   │   │   └─ 每 50ms 发布 Feedback (当前进度百分比)
   │   │
   │   └─→ dm_arm_hardware::write() [500Hz 循环]
   │       ├─ 读取 hw_commands_[i] (控制器写入的目标)
   │       ├─ 计算速度：v = (cmd - cmd_prev) / dt
   │       ├─ PID 补偿：v += kp * (cmd - measured)
   │       ├─ 限速：v = clamp(v, -3.0, 3.0)
   │       ├─ 下发 CAN 指令
   │       │   └─ motor_controller_->control_pos_vel(*motor, cmd, v)
   │       │       └─ 构造达妙协议帧，通过串口发送
   │       └─ 更新 cmd_prev
   │
   └─→ 轨迹执行完成后 Action 返回 Result
       ├─ error_code: SUCCESS / GOAL_TOLERANCE_VIOLATED / ...
       └─ error_string: 描述信息

5. MoveGroupInterface::move() 返回
   └─→ 应用层收到 moveit::core::MoveItErrorCode::SUCCESS
```

**关键时间节点：**
- 规划耗时：通常 0.5~2.0 秒（取决于路径复杂度）
- 轨迹执行：取决于路径长度和速度限制，dm_arm 典型 3~6 秒
- 控制周期：500Hz（2ms），确保平滑跟踪

---

### 1.3 类关系图

```
moveit::planning_interface::MoveGroupInterface
  ├─→ owns: MoveGroupImpl (pimpl 模式)
  │    ├─→ owns: rclcpp::Node::SharedPtr
  │    ├─→ owns: tf2_ros::Buffer
  │    ├─→ owns: planning_scene_monitor::PlanningSceneMonitorPtr
  │    ├─→ client: /plan_kinematic_path (Service)
  │    ├─→ client: /compute_cartesian_path (Service)
  │    ├─→ action_client: /execute_trajectory (Action)
  │    └─→ subscriber: /move_group/monitored_planning_scene
  │
  └─→ 成员变量（public API 可访问）:
       ├─ std::string group_name_
       ├─ std::string planning_frame_
       ├─ std::string end_effector_link_
       ├─ moveit_msgs::msg::Constraints path_constraints_
       └─ moveit::core::RobotStatePtr remembered_joint_values_

moveit::core::RobotModel (从 URDF + SRDF 解析)
  ├─→ owns: std::vector<JointModelGroup*>
  │    └─ JointModelGroup ("arm")
  │         ├─ std::vector<JointModel*> active_joints_
  │         │    ├─ RevoluteJointModel (joint1)
  │         │    ├─ RevoluteJointModel (joint2)
  │         │    └─ ... (joint3-6)
  │         ├─ std::vector<LinkModel*> links_
  │         │    ├─ LinkModel (base_link)
  │         │    ├─ LinkModel (link1-2)
  │         │    └─ ... (所有 arm 组的连杆)
  │         ├─ kinematics::KinematicsBasePtr solver_instance_
  │         │    └─ kdl_kinematics_plugin::KDLKinematicsPlugin
  │         │         ├─ KDL::Chain (运动学链)
  │         │         ├─ KDL::ChainFkSolverPos_recursive (FK 求解器)
  │         │         └─ KDL::ChainIkSolverPos_LMA (IK 求解器)
  │         └─ std::map<std::string, std::vector<double>> named_targets_
  │              └─ "zero" → [0, 0, 0, 0, 0, 0]
  │
  └─→ owns: srdf::Model (SRDF 数据)
       ├─ std::vector<srdf::Model::EndEffector> end_effectors_
       ├─ std::vector<srdf::Model::Group> groups_
       ├─ std::vector<srdf::Model::GroupState> group_states_
       └─ collision_detection::AllowedCollisionMatrix acm_

moveit::core::RobotState (某一时刻的机器人状态)
  ├─→ reference: RobotModel* (指向上述 RobotModel)
  ├─→ owns: std::vector<double> position_ (所有关节的位置值)
  ├─→ owns: std::vector<double> velocity_
  ├─→ owns: std::vector<double> effort_
  ├─→ owns: std::map<std::string, Eigen::Isometry3d> global_link_transforms_
  │    └─ "link_tcp" → 4×4 变换矩阵 (从 world/base_link 到 link_tcp)
  │
  └─→ 方法:
       ├─ setJointGroupPositions(group, values)
       ├─ copyJointGroupPositions(group, values)
       ├─ bool setFromIK(jmg, pose, timeout) ← 调用 IK 求解器
       ├─ getGlobalLinkTransform(link_name) ← FK 计算
       └─ updateLinkTransforms() ← 刷新所有连杆变换矩阵

planning_scene_monitor::PlanningSceneMonitor
  ├─→ owns: planning_scene::PlanningScenePtr scene_
  │    └─ planning_scene::PlanningScene
  │         ├─→ reference: RobotModelConstPtr
  │         ├─→ owns: RobotStatePtr current_state_
  │         ├─→ owns: collision_detection::WorldPtr world_
  │         │    └─ std::map<std::string, shapes::ShapeConstPtr> objects_
  │         │         ├─ "table" → Box(0.6, 1.2, 0.02)
  │         │         └─ "post" → Cylinder(1.0, 0.05)
  │         └─→ owns: collision_detection::AllowedCollisionMatrix acm_
  │
  ├─→ subscriber: /joint_states
  │    └─ 回调: updateSceneWithCurrentState()
  │         └─ scene_->getCurrentStateNonConst().setVariableValues(joint_state)
  │
  ├─→ subscriber: /planning_scene
  │    └─ 回调: newPlanningSceneMessage()
  │         └─ scene_->usePlanningSceneMsg(msg) ← 差量更新
  │
  ├─→ subscriber: /collision_object
  │    └─ 回调: collisionObjectCallback()
  │         └─ scene_->processCollisionObjectMsg(msg)
  │
  └─→ owns: tf2_ros::Buffer
       └─ 用于 TF 查询（坐标系变换）

hardware_interface::SystemInterface (dm_arm_hardware 实现的基类)
  ├─→ 虚函数 (必须实现):
  │    ├─ CallbackReturn on_init(HardwareInfo)
  │    ├─ CallbackReturn on_configure(State)
  │    ├─ CallbackReturn on_activate(State)
  │    ├─ CallbackReturn on_deactivate(State)
  │    ├─ std::vector<StateInterface> export_state_interfaces()
  │    ├─ std::vector<CommandInterface> export_command_interfaces()
  │    ├─ return_type read(time, period)
  │    └─ return_type write(time, period)
  │
  └─→ 成员变量:
       └─ HardwareInfo info_ (从 URDF <ros2_control> 解析)
            ├─ name: "dm_arm_hardware"
            ├─ hardware_parameters: {"serial_port": "/dev/ttyACM0", ...}
            └─ joints: [
                 {name: "joint1", parameters: {"motor_id": "1", ...}},
                 {name: "joint2", parameters: {"motor_id": "2", ...}},
                 ...
               ]

controller_interface::ControllerInterface
  └─ joint_trajectory_controller::JointTrajectoryController
       ├─→ owns: std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_points_
       ├─→ owns: rclcpp_action::Server<FollowJointTrajectory>
       ├─→ reference: std::vector<LoanedCommandInterface> cmd_ifaces_
       │    └─ 指向 dm_arm_hardware 导出的 hw_commands_[i]
       └─→ reference: std::vector<LoanedStateInterface> state_ifaces_
            └─ 指向 dm_arm_hardware 导出的 hw_positions_[i]
```

---

## 第二章：从最小可用代码开始

### 2.1 目标

在 `dm_arm_moveit_config` 的 `demo.launch.py` 基础上，编写第一个运动规划程序：

**功能：让机械臂移动到指定的关节角度**

---

### 2.2 创建 C++ 节点包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake dm_arm_examples \
    --dependencies rclcpp moveit_ros_planning_interface
```

目录结构：
```
dm_arm_examples/
├── CMakeLists.txt
├── package.xml
└── src/
    └── 01_move_to_joint_target.cpp  ← 第一个示例
```

---

### 2.3 最小可用代码

`src/01_move_to_joint_target.cpp`：

```cpp
#include "dm_arm_controller/end_effector_cmd.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void shutdown_thread(std::thread& spin_thread);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("end_effector_cmd");
    RCLCPP_INFO(node->get_logger(), "节点：end_effector_cmd 已启动");

    // 需要让开线程让节点 spin 来实时更新状态
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 创建 MoveGroupInterface 对象，指定控制的机械臂组名称
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    RCLCPP_INFO(node->get_logger(), "Planning Frame - %s 已创建", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End Effector Link - %s 已创建", arm.getEndEffectorLink().c_str());

    // 获取当前关节值并打印
    std::vector<double> current_joints = arm.getCurrentJointValues();
    std::vector<std::string> joint_names = arm.getJointNames();
    RCLCPP_INFO(node->get_logger(), "当前关节值：");
    for(size_t i = 0; i < current_joints.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "%s: %f", joint_names[i].data(), current_joints[i]);
    }

    // 设置目标关节值
    std::vector<double> target_joints = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    bool success = arm.setJointValueTarget(target_joints);
    RCLCPP_INFO(node->get_logger(), "设置目标关节值是否成功：%s", success ? "是" : "否");
    if(!success) {
        shutdown_thread(spin_thread);
        return 1;
    }

    // 规划
    RCLCPP_INFO(node->get_logger(), "正在规划...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode err_code = arm.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "规划失败，错误码：%d", err_code.val);
        shutdown_thread(spin_thread);
        return 1;
    }

    // 执行
    RCLCPP_INFO(node->get_logger(), "规划成功，正在执行...");
    err_code = arm.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "执行失败，错误码：%d", err_code.val);
        shutdown_thread(spin_thread);
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "执行成功，目标位置已达成");
    shutdown_thread(spin_thread);

    return 0;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 安全地关闭 ROS 2 节点并等待 spin 线程结束
 * @param spin_thread 负责节点 spin 的线程
 */
static void shutdown_thread(std::thread& spin_thread) {
    rclcpp::shutdown();
    if(spin_thread.joinable()) {
        spin_thread.join();
    }
}

```

---

### 2.4 CMakeLists.txt 配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(dm_arm_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 可执行文件
add_executable(01_move_to_joint_target src/01_move_to_joint_target.cpp)
ament_target_dependencies(01_move_to_joint_target
  rclcpp
  moveit_ros_planning_interface
)

# 安装
install(TARGETS 01_move_to_joint_target
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

### 2.5 编译与运行

```bash
# 构建
cd ~/ros2_ws
colcon build --symlink-install --packages-select dm_arm_examples
source install/setup.bash

# 启动 MoveIt2（仿真模式）
ros2 launch dm_arm_moveit_config demo.launch.py

# 另开终端，运行示例
ros2 run dm_arm_examples 01_move_to_joint_target
```

**预期输出：**
```
[INFO] [move_to_joint_target]: Node started: move_to_joint_target
[INFO] [move_to_joint_target]: Planning frame: base_link
[INFO] [move_to_joint_target]: End effector: link_tcp
[INFO] [move_to_joint_target]: Current joints:
[INFO] [move_to_joint_target]:   joint1: 0.000 rad (0.0 deg)
[INFO] [move_to_joint_target]:   joint2: 0.000 rad (0.0 deg)
[INFO] [move_to_joint_target]:   joint3: 0.000 rad (0.0 deg)
[INFO] [move_to_joint_target]:   joint4: 0.000 rad (0.0 deg)
[INFO] [move_to_joint_target]:   joint5: 0.000 rad (0.0 deg)
[INFO] [move_to_joint_target]:   joint6: 0.000 rad (0.0 deg)
[INFO] [move_to_joint_target]: Planning to target...
[INFO] [move_to_joint_target]: Planning succeeded! Planning time: 0.82 sec
[INFO] [move_to_joint_target]: Trajectory has 18 waypoints
[INFO] [move_to_joint_target]: Executing trajectory...
[INFO] [move_to_joint_target]: Execution succeeded!
```

**RViz 中观察：**
- Planning Request 面板会显示目标状态（橙色虚影）
- 执行时机械臂平滑移动到目标关节角度

---

### 2.6 代码逐行讲解

#### 2.6.1 为什么需要 Node？

```cpp
auto node = rclcpp::Node::make_shared("move_to_joint_target");
moveit::planning_interface::MoveGroupInterface arm(node, "arm");
```

**ROS1 与 ROS2 的关键差异：**

| ROS1 | ROS2 |
|------|------|
| `moveit::planning_interface::MoveGroupInterface arm("arm");`<br>不需要传 NodeHandle | `MoveGroupInterface arm(node, "arm");`<br>**必须传 Node** |
| 内部创建全局 NodeHandle | 内部使用传入的 node 进行所有 ROS 通信 |

**原因：** ROS2 没有全局 NodeHandle 概念，所有 ROS 通信（Service、Action、Subscriber）都必须绑定到具体的 Node 对象。

MoveGroupInterface 内部需要：
- 订阅 `/joint_states`（获取当前关节状态）
- 订阅 `/planning_scene`（场景更新）
- 调用 `/plan_kinematic_path` 服务（规划）
- 调用 `/execute_trajectory` Action（执行）

这些都需要 Node 提供的通信基础设施。

---

#### 2.6.2 getPlanningFrame() 与 getEndEffectorLink()

```cpp
arm.getPlanningFrame()      // 返回: "base_link"
arm.getEndEffectorLink()    // 返回: "link_tcp"
```

**来源：**
1. `getPlanningFrame()` 从 URDF 的 `<link name="base_link">` 推断（通常是运动学链的根连杆）
2. `getEndEffectorLink()` 从 SRDF 的两处之一获取：
   - **方式一（推荐）：** `<end_effector name="gripper" parent_link="arm" group="gripper"/>`
     - 此时返回 gripper 组的 tip link
   - **方式二：** `<group name="arm"> <chain base_link="base_link" tip_link="link_tcp"/> </group>`
     - 若无 end_effector 定义，返回 chain 的 tip_link

**dm_arm 的实际配置：**
```xml
<!-- dm_arm.srdf -->
<group name="arm">
  <chain base_link="base_link" tip_link="link_tcp"/>
</group>
```
因此 `getEndEffectorLink()` 返回 `"link_tcp"`。

---

#### 2.6.3 setJointValueTarget() 的顺序约定

```cpp
std::vector<double> target_joints = {0.0, 0.5, 1.0, 0.0, 0.0, 0.0};
arm.setJointValueTarget(target_joints);
```

**关键：向量顺序必须与 `getJointNames()` 一致！**

验证顺序：
```cpp
std::vector<std::string> names = arm.getJointNames();
for (const auto& name : names) {
  std::cout << name << std::endl;
}
// 输出:
// joint1
// joint2
// joint3
// joint4
// joint5
// joint6
```

**错误示例：**
```cpp
// 错误！顺序不对应
std::vector<double> wrong_order = {
  0.0,  // 本意是 joint1，但实际会被赋给 getJointNames()[0]（确实是 joint1，这里恰好对了）
  1.0,  // 本意是 joint3，但实际被赋给 joint2 ← 错误！
  0.5   // 本意是 joint2，但实际被赋给 joint3 ← 错误！
};
```

正确做法：始终按 `getJointNames()` 返回的顺序填充向量。

---

#### 2.6.4 plan() 返回值详解

```cpp
moveit::planning_interface::MoveGroupInterface::Plan plan;
moveit::core::MoveItErrorCode error_code = arm.plan(plan);
```

**Plan 结构体成员：**
```cpp
struct Plan {
  moveit_msgs::msg::RobotState start_state_;
  // 起始状态：包含所有关节的初始位置

  moveit_msgs::msg::RobotTrajectory trajectory_;
  // 轨迹：核心数据，包含 joint_trajectory 和 multi_dof_joint_trajectory
  // 对于串联机械臂，只有 joint_trajectory 有数据

  double planning_time_;
  // 规划耗时（秒）
};
```

**trajectory_.joint_trajectory 结构：**
```cpp
trajectory_msgs::msg::JointTrajectory {
  std::vector<std::string> joint_names;
  // ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

  std::vector<JointTrajectoryPoint> points;
  // 轨迹点数组，每个点包含：
  //   - positions: [j1, j2, j3, j4, j5, j6]（弧度）
  //   - velocities: [v1, v2, v3, v4, v5, v6]（rad/s）
  //   - accelerations: [a1, a2, a3, a4, a5, a6]（rad/s²）
  //   - time_from_start: Duration（从轨迹起点算起的时间）
}
```

**示例轨迹点：**
```
points[0]:
  positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start = 0.0s

points[5]:
  positions = [0.0, 0.25, 0.5, 0.0, 0.0, 0.0]
  velocities = [0.0, 0.3, 0.6, 0.0, 0.0, 0.0]
  time_from_start = 1.2s

points[17] (最后一个点):
  positions = [0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
  velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  ← 终点速度为零
  time_from_start = 3.8s
```

**MoveItErrorCode 常用值：**
```cpp
moveit::core::MoveItErrorCode::SUCCESS              // 1
moveit::core::MoveItErrorCode::PLANNING_FAILED      // -1
moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN  // -2
moveit::core::MoveItErrorCode::TIMED_OUT            // -6
moveit::core::MoveItErrorCode::NO_IK_SOLUTION       // -31
```

判断成功的标准方式：
```cpp
if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
  // 规划成功
}
```

---

#### 2.6.5 execute() 的内部流程

```cpp
moveit::core::MoveItErrorCode error_code = arm.execute(plan);
```

execute() 做了什么：
1. **验证轨迹：** 检查轨迹起点是否与当前状态接近（误差 < 0.01 rad）
   - 若不接近，返回 `START_STATE_INVALID`
2. **查找控制器：** 根据 `moveit_controllers.yaml` 确定 `arm_controller` 管理这 6 个关节
3. **发送 Action：** 调用 `/arm_controller/follow_joint_trajectory` Action
4. **等待完成：** 阻塞直到轨迹执行完成或超时

**超时设置：**
```cpp
arm.setExecutionTimeout(15.0);  // 默认 15 秒
```

若轨迹执行超过 15 秒，返回 `TIMED_OUT`。

---

### 2.7 常见问题排查

#### 问题 1：`Failed to construct MoveGroup`

**症状：**
```
terminate called after throwing an instance of 'std::runtime_error'
  what():  Failed to construct MoveGroup
```

**原因：** `move_group` 节点未启动或崩溃。

**排查：**
```bash
# 检查 move_group 是否在运行
ros2 node list | grep move_group

# 若不存在，检查 launch 文件
ros2 launch dm_arm_moveit_config demo.launch.py
```

---

#### 问题 2：`Planning failed with error code: -1`

**症状：**
```
[ERROR] [move_to_joint_target]: Planning failed with error code: -1
```

**原因：** 目标关节值超出限位或起点碰撞。

**排查：**
```bash
# 检查关节限位（来自 URDF 和 joint_limits.yaml）
ros2 param get /move_group robot_description_planning

# 输出示例：
# joint_limits:
#   joint1:
#     has_position_limits: true
#     min_position: -2.094
#     max_position: 2.094
```

确认 `target_joints` 中的每个值都在对应关节的 `[min, max]` 范围内。

---

#### 问题 3：`Execution failed with error code: -7`

**症状：**
```
[ERROR] [move_to_joint_target]: Execution failed with error code: -7
```

error_code = -7 对应 `CONTROL_FAILED`。

**原因：** 控制器未激活或超时。

**排查：**
```bash
# 检查控制器状态
ros2 control list_controllers

# 预期输出：
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# 若显示 inactive，手动激活：
ros2 control set_controller_state arm_controller activate
```

---

## 第三章：扩展功能 - 笛卡尔空间规划

### 3.1 目标

在上一节的基础上，增加新功能：**让机械臂末端移动到指定的 XYZ 位置（不限制姿态）**

---

### 3.2 新增文件

`src/02_move_to_position_target.cpp`：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_to_position_target");

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");

  RCLCPP_INFO(node->get_logger(), "=== Position-only Target Example ===");

  // ── 获取当前末端位置 ─────────────────────────────────────────
  geometry_msgs::msg::PoseStamped current_pose = arm.getCurrentPose();
  RCLCPP_INFO(node->get_logger(), "Current end-effector position:");
  RCLCPP_INFO(node->get_logger(), "  x: %.3f m", current_pose.pose.position.x);
  RCLCPP_INFO(node->get_logger(), "  y: %.3f m", current_pose.pose.position.y);
  RCLCPP_INFO(node->get_logger(), "  z: %.3f m", current_pose.pose.position.z);

  // ── 设置仅位置目标（姿态自由）─────────────────────────────────
  // API: setPositionTarget(x, y, z, end_effector_link)
  // 不设置姿态约束，IK 求解器会找到任意满足位置要求的解
  bool success = arm.setPositionTarget(
    0.3,   // x: 前方 30cm
    0.0,   // y: 正前方（无偏移）
    0.4    // z: 离地面 40cm
  );

  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set position target");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Target position: (0.3, 0.0, 0.4)");

  // ── 规划与执行 ────────────────────────────────────────────────
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode error_code = arm.plan(plan);

  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed: error_code=%d", error_code.val);

    // 常见失败原因：
    if (error_code.val == -31) {  // NO_IK_SOLUTION
      RCLCPP_ERROR(node->get_logger(),
        "Target position out of workspace (no IK solution)");
    }

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded (%.2f sec, %zu waypoints)",
    plan.planning_time_, plan.trajectory_.joint_trajectory.points.size());

  error_code = arm.execute(plan);

  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed: error_code=%d", error_code.val);
    rclcpp::shutdown();
    return 1;
  }

  // ── 验证最终位置 ──────────────────────────────────────────────
  rclcpp::sleep_for(std::chrono::milliseconds(500));  // 等待关节状态更新
  geometry_msgs::msg::PoseStamped final_pose = arm.getCurrentPose();

  RCLCPP_INFO(node->get_logger(), "Final position:");
  RCLCPP_INFO(node->get_logger(), "  x: %.3f m", final_pose.pose.position.x);
  RCLCPP_INFO(node->get_logger(), "  y: %.3f m", final_pose.pose.position.y);
  RCLCPP_INFO(node->get_logger(), "  z: %.3f m", final_pose.pose.position.z);

  // 计算位置误差
  double dx = final_pose.pose.position.x - 0.3;
  double dy = final_pose.pose.position.y - 0.0;
  double dz = final_pose.pose.position.z - 0.4;
  double error = std::sqrt(dx*dx + dy*dy + dz*dz);

  RCLCPP_INFO(node->get_logger(), "Position error: %.4f m", error);

  rclcpp::shutdown();
  return 0;
}
```

---

### 3.3 API 详解：setPositionTarget()

```cpp
bool setPositionTarget(
    double x, double y, double z,
    const std::string & end_effector_link = "");
```

**参数：**
- `x, y, z`：目标位置（米），在 `getPlanningFrame()` 坐标系下
  - dm_arm: base_link 坐标系，原点在基座中心
- `end_effector_link`：若为空，使用 `getEndEffectorLink()` 的值

**与 setPoseTarget() 的区别：**

| API | 约束 | IK 难度 | 适用场景 |
|-----|------|---------|---------|
| `setPoseTarget(pose)` | 位置 + 姿态（6自由度） | 高 | 需要精确姿态（如插拔、书写） |
| `setPositionTarget(x,y,z)` | 仅位置（3自由度） | 低 | 只关心位置（如抓取、点触） |
| `setOrientationTarget(qx,qy,qz,qw)` | 仅姿态（3自由度） | 中 | 姿态调整（如相机对准） |

**内部工作流程：**
1. `setPositionTarget()` 创建一个 `PositionConstraint`：
   ```cpp
   moveit_msgs::msg::PositionConstraint pc;
   pc.link_name = "link_tcp";
   pc.target_point_offset = {0, 0, 0};  // 连杆上的参考点
   pc.constraint_region.primitives[0] = Sphere(radius=0.001);  // 允许 1mm 误差球
   pc.constraint_region.primitive_poses[0] = {x, y, z};
   ```

2. IK 求解器只需满足位置约束，姿态自由选择

3. KDL IK 求解器会从多个随机初值出发寻找解，返回第一个成功的解

**工作空间限制：**
dm_arm 的可达空间约为半径 0.1~0.6 米的半球形区域（基座上方）。

验证目标是否可达：
```cpp
// 计算目标与基座的距离
double distance = std::sqrt(x*x + y*y + z*z);

if (distance < 0.1 || distance > 0.6) {
  RCLCPP_WARN(node->get_logger(),
    "Target may be out of workspace (distance=%.3f m)", distance);
}
```

---

### 3.4 getCurrentPose() 详解

```cpp
geometry_msgs::msg::PoseStamped current_pose = arm.getCurrentPose();
```

**返回值结构：**
```cpp
geometry_msgs::msg::PoseStamped {
  std_msgs::msg::Header header;
  // header.frame_id = "base_link" (getPlanningFrame() 的值)
  // header.stamp = 当前时间戳

  geometry_msgs::msg::Pose pose;
  // pose.position = {x, y, z} (米)
  // pose.orientation = {x, y, z, w} (四元数，归一化)
}
```

**内部实现：**
1. 从 `/joint_states` 获取当前关节值
2. 调用 FK（正运动学）计算末端位姿：
   ```cpp
   moveit::core::RobotStatePtr state = arm.getCurrentState();
   const Eigen::Isometry3d& tf = state->getGlobalLinkTransform("link_tcp");
   // tf 是从 base_link 到 link_tcp 的 4×4 变换矩阵
   ```
3. 提取位置和四元数

**注意事项：**
- `getCurrentState(timeout)` 会阻塞最多 `timeout` 秒等待 `/joint_states` 更新
- 若 joint_state_broadcaster 未启动，会返回 `nullptr` → `getCurrentPose()` 抛异常

---

### 3.5 编译与测试

在 `CMakeLists.txt` 中添加：
```cmake
add_executable(02_move_to_position_target src/02_move_to_position_target.cpp)
ament_target_dependencies(02_move_to_position_target
  rclcpp moveit_ros_planning_interface)
install(TARGETS 02_move_to_position_target
  DESTINATION lib/${PROJECT_NAME})
```

运行：
```bash
colcon build --packages-select dm_arm_examples
ros2 run dm_arm_examples 02_move_to_position_target
```

**预期输出：**
```
[INFO] [move_to_position_target]: Current end-effector position:
[INFO] [move_to_position_target]:   x: 0.466 m
[INFO] [move_to_position_target]:   y: 0.000 m
[INFO] [move_to_position_target]:   z: 0.100 m
[INFO] [move_to_position_target]: Target position: (0.3, 0.0, 0.4)
[INFO] [move_to_position_target]: Planning succeeded (1.23 sec, 22 waypoints)
[INFO] [move_to_position_target]: Final position:
[INFO] [move_to_position_target]:   x: 0.300 m
[INFO] [move_to_position_target]:   y: 0.000 m
[INFO] [move_to_position_target]:   z: 0.400 m
[INFO] [move_to_position_target]: Position error: 0.0012 m
```

---

## 第四章：完整位姿规划与 TF 变换

### 4.1 目标

进一步扩展：**设置完整的位置+姿态目标，并处理坐标系变换**

---

### 4.2 新增文件

`src/03_move_to_pose_target.cpp`：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_to_pose_target");

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");

  RCLCPP_INFO(node->get_logger(), "=== Full Pose Target Example ===");

  // ── 方式一：直接设置位姿（在 base_link 坐标系下）──────────────
  geometry_msgs::msg::Pose target_pose;

  // 位置：前方 35cm，右侧 10cm，高度 30cm
  target_pose.position.x = 0.35;
  target_pose.position.y = -0.10;
  target_pose.position.z = 0.30;

  // 姿态：末端垂直向下（Z 轴朝下）
  // 欧拉角：roll=0, pitch=90°, yaw=0
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI/2, 0.0);  // 单位：弧度
  q.normalize();
  target_pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(node->get_logger(), "Target pose in base_link frame:");
  RCLCPP_INFO(node->get_logger(), "  Position: (%.3f, %.3f, %.3f)",
    target_pose.position.x, target_pose.position.y, target_pose.position.z);
  RCLCPP_INFO(node->get_logger(), "  Orientation (RPY): (0.0, 90.0, 0.0) deg");

  // ── 设置目标 ─────────────────────────────────────────────────
  bool success = arm.setPoseTarget(target_pose);
  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set pose target");
    rclcpp::shutdown();
    return 1;
  }

  // ── 规划 ─────────────────────────────────────────────────────
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode error_code = arm.plan(plan);

  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed: %d", error_code.val);

    // 若 IK 失败，尝试放宽姿态约束
    if (error_code.val == -31) {
      RCLCPP_WARN(node->get_logger(),
        "Trying again with relaxed orientation tolerance...");

      // 增大姿态容差到 0.2 rad（约 11.5°）
      arm.setGoalOrientationTolerance(0.2);
      error_code = arm.plan(plan);

      if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Still failed after relaxing tolerance");
        rclcpp::shutdown();
        return 1;
      }
    } else {
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded!");

  // ── 执行 ─────────────────────────────────────────────────────
  error_code = arm.execute(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed: %d", error_code.val);
    rclcpp::shutdown();
    return 1;
  }

  // ── 验证最终姿态 ──────────────────────────────────────────────
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  geometry_msgs::msg::PoseStamped final_pose = arm.getCurrentPose();

  // 提取最终姿态的欧拉角
  tf2::Quaternion q_final;
  tf2::fromMsg(final_pose.pose.orientation, q_final);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_final).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(node->get_logger(), "Final pose:");
  RCLCPP_INFO(node->get_logger(), "  Position: (%.3f, %.3f, %.3f)",
    final_pose.pose.position.x,
    final_pose.pose.position.y,
    final_pose.pose.position.z);
  RCLCPP_INFO(node->get_logger(), "  Orientation (RPY): (%.1f, %.1f, %.1f) deg",
    roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);

  rclcpp::shutdown();
  return 0;
}
```

---

### 4.3 API 详解：setPoseTarget()

```cpp
bool setPoseTarget(const geometry_msgs::msg::Pose & pose,
                   const std::string & end_effector_link = "");
```

**约束类型：**

`setPoseTarget()` 内部创建两个约束：
1. **PositionConstraint**（位置约束）：
   ```cpp
   // 目标点必须在 (x±tol, y±tol, z±tol) 的立方体内
   // tol = getGoalPositionTolerance()，默认 0.0001 米
   ```

2. **OrientationConstraint**（姿态约束）：
   ```cpp
   // 目标姿态与实际姿态的旋转差必须 < getGoalOrientationTolerance()
   // 默认 0.001 弧度（约 0.057°）
   ```

**调整容差：**
```cpp
arm.setGoalPositionTolerance(0.01);      // 1cm 位置误差
arm.setGoalOrientationTolerance(0.1);    // 约 5.7° 姿态误差
```

容差越大，IK 成功率越高，但精度下降。

---

### 4.4 TF2 四元数操作详解

#### 4.4.1 RPY → 四元数

```cpp
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);  // 单位：弧度
q.normalize();  // 归一化（必须调用，否则可能不满足单位四元数条件）
```

**内旋顺序：** X-Y-Z（即先绕 X 轴 roll，再绕 Y 轴 pitch，最后绕 Z 轴 yaw）

**常用姿态：**

| 描述 | RPY (deg) | RPY (rad) |
|-----|----------|-----------|
| 末端水平向前（默认） | (0, 0, 0) | (0, 0, 0) |
| 末端垂直向下 | (0, 90, 0) | (0, π/2, 0) |
| 末端垂直向上 | (0, -90, 0) | (0, -π/2, 0) |
| 末端向左侧 | (0, 0, 90) | (0, 0, π/2) |
| 末端向右侧 | (0, 0, -90) | (0, 0, -π/2) |

---

#### 4.4.2 四元数 → RPY

```cpp
tf2::Quaternion q;
tf2::fromMsg(pose.orientation, q);  // geometry_msgs → tf2

double roll, pitch, yaw;
tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);  // 单位：弧度

// 转换为角度
double roll_deg = roll * 180.0 / M_PI;
double pitch_deg = pitch * 180.0 / M_PI;
double yaw_deg = yaw * 180.0 / M_PI;
```

---

#### 4.4.3 四元数乘法（旋转叠加）

**场景：** 在当前姿态基础上，绕末端 Z 轴旋转 45°

```cpp
// 获取当前姿态
geometry_msgs::msg::PoseStamped current = arm.getCurrentPose();
tf2::Quaternion q_current;
tf2::fromMsg(current.pose.orientation, q_current);

// 构造增量旋转（绕 Z 轴 45°）
tf2::Quaternion q_delta;
q_delta.setRPY(0, 0, 45.0 * M_PI / 180.0);

// 叠加旋转（注意顺序：先 current，后 delta）
tf2::Quaternion q_new = q_current * q_delta;
q_new.normalize();

// 应用到新目标
geometry_msgs::msg::Pose target = current.pose;
target.orientation = tf2::toMsg(q_new);

arm.setPoseTarget(target);
```

**注意：** 四元数乘法不满足交换律！`q1 * q2 ≠ q2 * q1`

---

### 4.5 编译与测试

```bash
# 添加到 CMakeLists.txt
add_executable(03_move_to_pose_target src/03_move_to_pose_target.cpp)
ament_target_dependencies(03_move_to_pose_target
  rclcpp moveit_ros_planning_interface tf2_geometry_msgs)
install(TARGETS 03_move_to_pose_target DESTINATION lib/${PROJECT_NAME})

# 构建
colcon build --packages-select dm_arm_examples

# 运行
ros2 run dm_arm_examples 03_move_to_pose_target
```

---

## 第五章：障碍物管理与碰撞检测

### 5.1 目标

添加环境障碍物，并验证 MoveIt2 的碰撞规避功能。

---

### 5.2 新增文件

`src/04_collision_avoidance.cpp`：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("collision_avoidance");

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  moveit::planning_interface::PlanningSceneInterface scene;

  RCLCPP_INFO(node->get_logger(), "=== Collision Avoidance Example ===");

  // ── 添加桌子（立方体障碍物）──────────────────────────────────
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "base_link";

  // 定义形状：立方体
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.6;   // X（宽）
  box.dimensions[1] = 1.2;   // Y（长）
  box.dimensions[2] = 0.02;  // Z（高）

  // 定义位姿：桌面在基座下方 1cm
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.01;
  box_pose.orientation.w = 1.0;

  // 将形状和位姿添加到碰撞物体
  table.primitives.push_back(box);
  table.primitive_poses.push_back(box_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  // 发布到场景
  scene.applyCollisionObject(table);

  RCLCPP_INFO(node->get_logger(), "Added table obstacle");

  // 等待场景更新（异步操作，需要时间）
  rclcpp::sleep_for(std::chrono::seconds(1));

  // ── 添加柱子（圆柱障碍物）─────────────────────────────────────
  moveit_msgs::msg::CollisionObject post;
  post.id = "post";
  post.header.frame_id = "base_link";

  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = cylinder.CYLINDER;
  cylinder.dimensions.resize(2);
  cylinder.dimensions[0] = 1.0;   // 高度（Z）
  cylinder.dimensions[1] = 0.05;  // 半径

  geometry_msgs::msg::Pose cyl_pose;
  cyl_pose.position.x = 0.3;
  cyl_pose.position.y = 0.2;
  cyl_pose.position.z = 0.5;
  cyl_pose.orientation.w = 1.0;

  post.primitives.push_back(cylinder);
  post.primitive_poses.push_back(cyl_pose);
  post.operation = moveit_msgs::msg::CollisionObject::ADD;

  scene.applyCollisionObject(post);

  RCLCPP_INFO(node->get_logger(), "Added post obstacle");
  rclcpp::sleep_for(std::chrono::seconds(1));

  // ── 设置目标（会经过柱子附近）─────────────────────────────────
  geometry_msgs::msg::Pose target;
  target.position.x = 0.3;
  target.position.y = 0.25;  // 靠近柱子
  target.position.z = 0.4;

  tf2::Quaternion q;
  q.setRPY(0, M_PI/2, 0);
  target.orientation = tf2::toMsg(q);

  arm.setPoseTarget(target);

  // ── 规划（自动避障）──────────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "Planning with collision avoidance...");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode error_code = arm.plan(plan);

  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed: %d", error_code.val);

    // 若目标本身在碰撞中
    if (error_code.val == -5) {  // GOAL_IN_COLLISION
      RCLCPP_ERROR(node->get_logger(),
        "Target pose collides with obstacles!");
    }

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded (avoided obstacles)");
  RCLCPP_INFO(node->get_logger(), "Trajectory has %zu waypoints",
    plan.trajectory_.joint_trajectory.points.size());

  // 执行
  error_code = arm.execute(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed: %d", error_code.val);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Motion completed!");

  // ── 清理障碍物 ────────────────────────────────────────────────
  std::vector<std::string> object_ids = {"table", "post"};
  scene.removeCollisionObjects(object_ids);

  RCLCPP_INFO(node->get_logger(), "Removed obstacles");

  rclcpp::shutdown();
  return 0;
}
```

---

### 5.3 API 详解：PlanningSceneInterface

#### 5.3.1 添加障碍物

```cpp
moveit::planning_interface::PlanningSceneInterface scene;
scene.applyCollisionObject(collision_object);
```

**CollisionObject 结构：**
```cpp
moveit_msgs::msg::CollisionObject {
  std::string id;                       // 唯一标识符
  std_msgs::msg::Header header;         // 坐标系
  std::vector<shape_msgs::msg::SolidPrimitive> primitives;  // 几何形状
  std::vector<geometry_msgs::msg::Pose> primitive_poses;    // 对应位姿
  uint8_t operation;                    // ADD/REMOVE/APPEND/MOVE
}
```

**支持的形状：**
```cpp
shape_msgs::msg::SolidPrimitive::BOX;        // 立方体：dimensions[x, y, z]
shape_msgs::msg::SolidPrimitive::SPHERE;     // 球体：dimensions[radius]
shape_msgs::msg::SolidPrimitive::CYLINDER;   // 圆柱：dimensions[height, radius]
shape_msgs::msg::SolidPrimitive::CONE;       // 圆锥：dimensions[height, radius]
```

---

#### 5.3.2 批量操作

```cpp
std::vector<moveit_msgs::msg::CollisionObject> objects;
objects.push_back(table);
objects.push_back(post);

// 一次性添加多个
scene.applyCollisionObjects(objects);

// 移除多个
scene.removeCollisionObjects({"table", "post"});
```

---

#### 5.3.3 查询场景

```cpp
// 获取所有物体
std::map<std::string, moveit_msgs::msg::CollisionObject> all_objects =
  scene.getObjects();

for (const auto & [id, obj] : all_objects) {
  RCLCPP_INFO(node->get_logger(), "Object: %s", id.c_str());
}

// 获取特定物体
std::vector<std::string> ids = {"table"};
std::map<std::string, moveit_msgs::msg::CollisionObject> subset =
  scene.getObjects(ids);
```

---

### 5.4 碰撞检测原理

当调用 `arm.plan()` 时，MoveIt2 在每次扩展 RRT 树时都会调用碰撞检测：

```
1. 采样新配置 q_rand
2. 找到 RRT 树中最近的节点 q_near
3. 向 q_rand 方向扩展一小步 → q_new
4. 碰撞检测：
   ├─ 正运动学：计算 q_new 下所有连杆的位姿
   ├─ 检查自碰撞：link1 vs link2, link1 vs link3, ...
   │   └─ 跳过 SRDF <disable_collisions> 中豁免的碰撞对
   ├─ 检查环境碰撞：每个连杆 vs 每个障碍物
   │   ├─ 使用 FCL 库（Flexible Collision Library）
   │   └─ 碰撞检测算法：GJK（Gilbert-Johnson-Keerthi）
   └─ 若无碰撞，接受 q_new；否则丢弃
5. 重复直到找到路径或超时
```

**碰撞检测的性能瓶颈：**
- 每次检测约 0.1~1 ms（取决于连杆和障碍物数量）
- RRTConnect 通常需要扩展 500~2000 次
- 总碰撞检测时间占规划时间的 60%~80%

---

### 5.5 编译与测试

```bash
# CMakeLists.txt 添加
add_executable(04_collision_avoidance src/04_collision_avoidance.cpp)
ament_target_dependencies(04_collision_avoidance
  rclcpp moveit_ros_planning_interface)
install(TARGETS 04_collision_avoidance DESTINATION lib/${PROJECT_NAME})

# 构建运行
colcon build --packages-select dm_arm_examples
ros2 run dm_arm_examples 04_collision_avoidance
```

**RViz 中观察：**
- 障碍物显示为半透明立方体/圆柱
- 规划的路径会绕过障碍物（而非直线）

---

## 第六章：ros2_control 硬件接口实现

### 6.1 目标

实现 `dm_arm_hardware` 插件，将 MoveIt2 的轨迹下发到达妙电机。

---

### 6.2 关键文件结构

```
dm_arm_hardware/
├── CMakeLists.txt
├── package.xml
├── dm_arm_hardware.xml          ← pluginlib 描述
├── include/dm_arm_hardware/
│   ├── dm_hardware_interface.hpp
│   ├── SerialPort.h             ← 复用 ROS1 版本
│   └── damiao.h                 ← 复用 ROS1 版本
└── src/
    ├── dm_hardware_interface.cpp
    ├── SerialPort.cpp
    └── damiao.cpp
```

---

### 6.3 HardwareInterface 头文件

`include/dm_arm_hardware/dm_hardware_interface.hpp`：

```cpp
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// 复用 ROS1 的底层驱动
#include "dm_arm_hardware/SerialPort.h"
#include "dm_arm_hardware/damiao.h"

namespace dm_arm_hardware
{

class DmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DmHardwareInterface)

  // ── 生命周期回调 ───────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── 接口导出 ───────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── 读写 ───────────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void return_zero_smooth();

  // 配置参数（从 URDF <ros2_control><hardware><param> 读取）
  std::string serial_port_;
  int baudrate_;
  double control_frequency_;
  bool use_mit_mode_;
  double kp_, kd_;
  double max_position_change_;
  double max_velocity_;

  // 每个关节的配置（从 URDF <joint><param> 读取）
  struct JointConfig {
    std::string name;
    int motor_id;
    int motor_type;
    double lead = 0.0;  // 直线导轨：m/rad 换算比
  };
  std::vector<JointConfig> joint_configs_;

  // 状态/命令缓冲区（与 StateInterface/CommandInterface 共享内存）
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_prev_;

  // 达妙驱动
  std::shared_ptr<SerialPort> serial_;
  std::shared_ptr<damiao::Motor_Control> motor_controller_;
  std::vector<std::shared_ptr<damiao::Motor>> motors_;
};

}  // namespace dm_arm_hardware
```

---

### 6.4 on_init() 实现

```cpp
hardware_interface::CallbackReturn DmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用基类
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── 读取硬件级参数（info_.hardware_parameters 是 map<string,string>）──
  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
  
  baudrate_ = info_.hardware_parameters.count("baudrate")
    ? std::stoi(info_.hardware_parameters.at("baudrate")) : 921600;
  
  control_frequency_ = info_.hardware_parameters.count("control_frequency")
    ? std::stod(info_.hardware_parameters.at("control_frequency")) : 500.0;
  
  use_mit_mode_ = info_.hardware_parameters.count("use_mit_mode")
    ? (info_.hardware_parameters.at("use_mit_mode") == "true") : false;
  
  kp_ = info_.hardware_parameters.count("kp")
    ? std::stod(info_.hardware_parameters.at("kp")) : 30.0;
  
  kd_ = info_.hardware_parameters.count("kd")
    ? std::stod(info_.hardware_parameters.at("kd")) : 1.0;
  
  max_position_change_ = info_.hardware_parameters.count("max_position_change")
    ? std::stod(info_.hardware_parameters.at("max_position_change")) : 0.5;
  
  max_velocity_ = info_.hardware_parameters.count("max_velocity")
    ? std::stod(info_.hardware_parameters.at("max_velocity")) : 3.0;

  // ── 解析每个关节配置（info_.joints 是 vector<ComponentInfo>）────────
  for (const auto & joint : info_.joints) {
    JointConfig cfg;
    cfg.name = joint.name;
    
    cfg.motor_id = joint.parameters.count("motor_id")
      ? std::stoi(joint.parameters.at("motor_id")) : 0;
    
    cfg.motor_type = joint.parameters.count("motor_type")
      ? std::stoi(joint.parameters.at("motor_type")) : 0;
    
    cfg.lead = joint.parameters.count("lead")
      ? std::stod(joint.parameters.at("lead")) : 0.0;

    joint_configs_.push_back(cfg);

    // 验证接口配置
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
        "Joint '%s' must have exactly one 'position' command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 分配缓冲区
  size_t n = joint_configs_.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);
  hw_commands_prev_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"),
    "on_init OK: %zu joints", n);

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

**关键点：**
1. `info_.hardware_parameters` 来自 URDF `<hardware><param name="..." value="..."/>`
2. `info_.joints[i].parameters` 来自 URDF `<joint><param name="..." value="..."/>`
3. 参数全是字符串，需手动转换（`std::stoi`/`std::stod`）
4. 缓冲区必须在 `on_init` 中分配且**不能再改变大小**

---

### 6.5 export_state_interfaces() 实现

```cpp
std::vector<hardware_interface::StateInterface>
DmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const std::string & name = joint_configs_[i].name;

    // StateInterface(joint_name, interface_type, pointer_to_data)
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}
```

**工作原理：**
- 返回的 `StateInterface` 包含指向 `hw_positions_[i]` 的指针
- `controller_manager` 将这些指针传递给控制器
- 控制器通过指针读取 `hw_positions_[i]` 的值
- `read()` 函数负责更新 `hw_positions_[i]`

**内存约束：**
`hw_positions_` 的内存地址在整个节点生命周期内不能改变！

```cpp
// 错误示例
void some_function() {
  hw_positions_.push_back(0.0);  // ❌ 可能触发重新分配，导致指针失效
}

// 正确做法
on_init() {
  hw_positions_.assign(n, 0.0);  // 一次性分配，之后不再改变大小
}
```

---

### 6.6 write() 实现（核心）

```cpp
hardware_interface::return_type DmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) dt = 1.0 / control_frequency_;

  for (size_t i = 0; i < motors_.size(); ++i) {
    try {
      // ── 单位换算（gripper_left 是直线导轨）──────────────────────
      const bool is_gripper = (joint_configs_[i].lead > 0.0);
      const double scale_to_motor = is_gripper
        ? (2.0 * M_PI / joint_configs_[i].lead) : 1.0;

      double cmd_motor  = hw_commands_[i]      * scale_to_motor;
      double prev_motor = hw_commands_prev_[i] * scale_to_motor;
      double meas_motor = hw_positions_[i]     * scale_to_motor;

      // ── 安全限制：单步最大位置变化 ─────────────────────────────
      double position_change = cmd_motor - prev_motor;
      if (std::abs(position_change) > max_position_change_) {
        position_change = std::copysign(max_position_change_, position_change);
        cmd_motor = prev_motor + position_change;
      }

      // ── 计算目标速度 ───────────────────────────────────────────
      double target_vel;
      if (use_mit_mode_) {
        // MIT 模式：前馈速度
        target_vel = position_change / dt;
      } else {
        // 位置速度模式：PID 跟踪 + 前馈
        double pos_err = cmd_motor - meas_motor;
        target_vel = 10.0 * pos_err;  // P 系数
        if (std::abs(position_change) > 1e-4) {
          target_vel += position_change / dt;
        }
      }

      // ── 限速 ───────────────────────────────────────────────────
      target_vel = std::clamp(target_vel, -max_velocity_, max_velocity_);

      // ── 下发 CAN 指令 ──────────────────────────────────────────
      if (use_mit_mode_ || i == 0) {
        // joint1 固定用 MIT 模式（硬件限制）
        motor_controller_->control_mit(
          *motors_[i], kp_, kd_, cmd_motor, target_vel, 0.0f);
      } else {
        motor_controller_->control_pos_vel(
          *motors_[i], cmd_motor, target_vel);
      }

      // ── 更新历史命令 ───────────────────────────────────────────
      hw_commands_prev_[i] = cmd_motor / scale_to_motor;

    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DmHardwareInterface"),
        *rclcpp::Clock::make_shared(), 1000,
        "Write motor %s error: %s", joint_configs_[i].name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}
```

**关键设计点：**

1. **单位换算：** gripper_left 的 `lead = 0.053` 表示电机每转一圈（2π rad），导轨移动 0.053 米。
   - 控制器发送的 `hw_commands_[i]` 单位是米
   - 电机需要的是弧度
   - 换算公式：`cmd_motor = hw_commands_[i] * (2π / 0.053)`

2. **安全限制：** 防止控制器发送跳变指令（如从 0 突变到 3.14）
   - 限制单步变化 < 0.5 rad（约 28.6°）
   - 按 500Hz 频率，最大角速度 = 0.5 × 500 = 250 rad/s（远超关节限速 3 rad/s，实际会被 `max_velocity_` 限制）

3. **速度计算：**
   - **MIT 模式（前馈）：** `v = Δpos / Δt`
   - **位置速度模式（PID + 前馈）：** `v = kp * error + Δpos / Δt`
     - `kp=10` 表示位置误差 0.1 rad → 速度 1 rad/s

4. **限速：** 确保速度不超过 `joint_limits.yaml` 中的 `max_velocity`（经 `velocity_scaling_factor` 缩放后）

---

### 6.7 完整的构建与测试流程

#### 6.7.1 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(dm_arm_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(dm_arm_hardware SHARED
  src/dm_hardware_interface.cpp
  src/SerialPort.cpp
  src/damiao.cpp
)

target_include_directories(dm_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_arm_hardware
  rclcpp hardware_interface pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface dm_arm_hardware.xml)

install(TARGETS dm_arm_hardware DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()
```

#### 6.7.2 pluginlib 描述文件

`dm_arm_hardware.xml`：
```xml
<library path="dm_arm_hardware">
  <class name="dm_arm_hardware/DmHardwareInterface"
         type="dm_arm_hardware::DmHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>dm_arm hardware interface using Damiao motors</description>
  </class>
</library>
```

#### 6.7.3 URDF 集成

在 `dm_arm_description/urdf/dm_arm.ros2_control.xacro` 中：

```xml
<ros2_control name="dm_arm_hardware" type="system">
  <hardware>
    <plugin>dm_arm_hardware/DmHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baudrate">921600</param>
    <param name="control_frequency">500.0</param>
    <param name="use_mit_mode">false</param>
    <param name="kp">30.0</param>
    <param name="kd">1.0</param>
    <param name="max_position_change">0.5</param>
    <param name="max_velocity">3.0</param>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">1</param>
    <param name="motor_type">2</param>  <!-- DM-J4340 -->
  </joint>

  <!-- joint2-6 类似 -->

  <joint name="gripper_left">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">7</param>
    <param name="motor_type">0</param>  <!-- DM-J4310 -->
    <param name="lead">0.053</param>    <!-- 导轨传动比 -->
  </joint>
</ros2_control>
```

#### 6.7.4 测试

```bash
# 构建
colcon build --packages-select dm_arm_hardware

# 检查插件是否注册成功
ros2 pkg list | grep dm_arm_hardware

# 仿真测试（不连接真实硬件）
ros2 launch dm_arm_moveit_config demo.launch.py use_fake_hardware:=true

# 真实硬件（需连接电机）
ros2 launch dm_arm_moveit_config demo.launch.py use_fake_hardware:=false
```

**验证硬件接口是否加载：**
```bash
ros2 control list_hardware_interfaces

# 预期输出：
# command interfaces:
#   joint1/position [available] [claimed]
#   joint2/position [available] [claimed]
#   ...
# state interfaces:
#   joint1/position [available]
#   joint1/velocity [available]
#   joint1/effort [available]
#   ...
```

## 进阶内容：

**规划与运动**

-   轨迹时间参数化：多种时间参数化算法（更平滑、更可控的速度/加速度）
-   笛卡尔路径规划：按直线路径/圆弧路径生成轨迹（对末端路径更直观）
-   轨迹后处理：平滑/滤波、轨迹修剪、约束验证

**环境感知与动态世界**

-   PlanningScene 管理：动态添加/移除障碍物、更新场景
-   传感器集成：Octomap / 点云融合用于动态避障
-   允许/禁止碰撞矩阵（ACM）精细控制

**约束与任务级规划**

-   目标/路径约束（姿态约束、位置约束、关节约束）
-   Task Constructor（MTC）：拼装“抓取-移动-放置”等任务级流程
-   多阶段规划与失败回滚策略

**执行与可靠性**

-   轨迹执行监控：实时反馈、失败恢复
-   异步执行与取消：支持中途 stop/cancel
-   控制器切换与状态验证

**交互与接口层**

-   MoveIt Servo：手柄/视觉闭环的实时伺服控制
-   规划接口服务化：MoveGroup 服务、Action 接口
-   可视化与调试：RViz 交互标记、规划可视化

---

## 附录：完整 API 速查

由于篇幅限制，完整的 API 参考请查阅配套的 `Moveit2 API Reference.md` 文档，包含：
- MoveGroupInterface 全部 50+ 个方法
- RobotState 的 FK/IK/Jacobian 计算
- PlanningScene 碰撞检测详解
- TF2 四元数操作完整示例
- ros2_control 生命周期状态机
- 常见报错与解决方案

---

## 总结

本教程从最小可用代码开始，逐步扩展功能：
1. **关节空间规划** → 理解 `setJointValueTarget()` 和 `plan()`/`execute()`
2. **笛卡尔位置规划** → 理解 `setPositionTarget()` 和 IK 求解
3. **完整位姿规划** → 掌握四元数操作和姿态约束
4. **障碍物管理** → 掌握 `PlanningSceneInterface` 和碰撞检测原理
5. **硬件接口实现** → 完整实现 `SystemInterface` 并理解 ros2_control 架构

每一步都包含：
- 完整可运行的代码
- 逐行讲解与 API 说明
- 内部工作原理剖析
- 常见问题与调试方法

通过本教程，你已具备从零构建 MoveIt2 应用的能力。
