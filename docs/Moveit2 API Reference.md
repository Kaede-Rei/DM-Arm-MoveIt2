# MoveIt2 完整 API 参考手册

> 配套 `Moveit2 Complete utorial.md` 的详细 API 文档
> 
> 本文档提供所有 MoveIt2 核心类的完整 API 说明、参数解释、使用示例和注意事项

---

[TOC]

---

## 第一部分：MoveGroupInterface 完整 API

### 1.1 构造与初始化

#### 1.1.1 构造函数

```cpp
moveit::planning_interface::MoveGroupInterface(
    const rclcpp::Node::SharedPtr & node,
    const std::string & group_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer = nullptr,
    const rclcpp::Duration & wait_for_servers = rclcpp::Duration::from_seconds(0.0)
);
```

**参数：**
- `node`：ROS2 节点的 shared_ptr，**必须传入**（ROS1 不需要此参数）
- `group_name`：规划组名称，必须在 SRDF 的 `<group name="...">` 中定义
  - dm_arm 示例：`"arm"` 或 `"gripper"`
- `tf_buffer`：可选的 TF2 Buffer，若为 `nullptr` 则内部自动创建
- `wait_for_servers`：等待 move_group Action 服务器的超时时间
  - `0.0` = 无限等待（默认）
  - 若超时会抛出 `std::runtime_error` 异常

**使用示例：**
```cpp
auto node = rclcpp::Node::make_shared("my_node");
moveit::planning_interface::MoveGroupInterface arm(node, "arm");
```

**注意事项：**
1. 构造函数会**阻塞**直到 `/move_group` 节点的 Action 服务器可达
2. 必须在节点 `spin()` 之前构造，但 `node` 必须已创建
3. 若 move_group 未启动，构造函数会一直阻塞（或超时后抛异常）

---

#### 1.1.2 基本信息查询

```cpp
const std::string & getName() const;
```
- **返回：** 规划组名称（构造时传入的 `group_name`）
- **示例：** `"arm"`

```cpp
const std::string & getPlanningFrame() const;
```
- **返回：** 规划参考坐标系名称
- **来源：** 从 URDF 推断，通常是运动学链的基座连杆
- **dm_arm：** `"base_link"`

```cpp
const std::string & getEndEffectorLink() const;
```
- **返回：** 末端执行器连杆名称
- **来源：**
  - 优先从 SRDF `<end_effector>` 获取
  - 若无，则从 `<group><chain tip_link="...">` 获取
- **dm_arm：** `"link_tcp"`

```cpp
const std::vector<std::string> & getJointNames() const;
```
- **返回：** 规划组内所有关节名称的有序列表
- **顺序：** 与运动学链顺序一致
- **dm_arm arm 组：** `["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]`
- **用途：** `setJointValueTarget()` 的参数顺序必须与此一致

```cpp
const std::vector<std::string> & getLinkNames() const;
```
- **返回：** 规划组内所有连杆名称
- **dm_arm arm 组：** `["base_link", "link1-2", "link1", "link2", "link4-5", "link5-6", "link6-7", "link_tcp"]`

```cpp
const std::vector<std::string> & getActiveJoints() const;
```
- **返回：** 主动关节（可控）名称列表
- **与 getJointNames() 的区别：** 排除固定关节（fixed joint）

```cpp
std::vector<double> getJointValueTarget() const;
```
- **返回：** 当前设置的关节空间目标值
- **顺序：** 与 `getJointNames()` 一致
- **用途：** 查询上次调用 `setJointValueTarget()` 设置的值

---

### 1.2 目标设置 API

#### 1.2.1 关节空间目标

```cpp
bool setJointValueTarget(const std::vector<double> & group_variable_values);
```
- **参数：** 目标关节值向量（单位：弧度），长度必须等于 `getJointNames().size()`
- **顺序：** 必须与 `getJointNames()` 返回顺序一致
- **返回：** `true` = 设置成功，`false` = 超出关节限位
- **示例：**
  ```cpp
  std::vector<double> target = {0.0, 0.5, 1.0, 0.0, 0.0, 0.0};
  if (!arm.setJointValueTarget(target)) {
    RCLCPP_ERROR(node->get_logger(), "Target out of joint limits");
  }
  ```

```cpp
bool setJointValueTarget(const std::string & joint_name, double value);
```
- **参数：** 
  - `joint_name`：关节名称（必须在规划组内）
  - `value`：目标值（弧度）
- **行为：** 只设置指定关节，其余关节保持当前值
- **返回：** `true` = 成功，`false` = 关节名不存在或值超限
- **示例：**
  ```cpp
  arm.setJointValueTarget("joint2", 1.0);
  ```

```cpp
bool setJointValueTarget(const moveit::core::RobotState & robot_state);
```
- **参数：** 一个完整的 RobotState 对象
- **行为：** 从 `robot_state` 提取规划组的关节值作为目标
- **用途：** 从之前保存的状态恢复
- **示例：**
  ```cpp
  auto state = arm.getCurrentState();
  state->setJointGroupPositions("arm", {0, 0, 0, 0, 0, 0});
  arm.setJointValueTarget(*state);
  ```

```cpp
bool setJointValueTarget(const sensor_msgs::msg::JointState & state);
```
- **参数：** JointState 消息
- **行为：** 从消息中提取规划组的关节值，忽略不属于规划组的关节
- **用途：** 从 `/joint_states` 话题或其他来源设置目标
- **注意：** 若消息中缺少规划组的某个关节，该关节保持当前值

```cpp
bool setJointValueTarget(const geometry_msgs::msg::Pose & eef_pose,
                          const std::string & end_effector_link = "");
```
- **参数：** 
  - `eef_pose`：末端位姿
  - `end_effector_link`：末端连杆名（空 = 使用 `getEndEffectorLink()`）
- **行为：** 调用 IK 求解器，将位姿转换为关节值
- **返回：** `true` = IK 成功，`false` = IK 失败
- **等价于：** `setPoseTarget(eef_pose)` 然后提取 IK 解

---

#### 1.2.2 命名目标（Named Target）

```cpp
bool setNamedTarget(const std::string & name);
```
- **参数：** 命名姿态的名称（来自 SRDF `<group_state name="...">`）
- **返回：** `true` = 存在该名称，`false` = 不存在
- **dm_arm SRDF 示例：**
  ```xml
  <group_state name="zero" group="arm">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0"/>
    <joint name="joint3" value="0"/>
    <joint name="joint4" value="0"/>
    <joint name="joint5" value="0"/>
    <joint name="joint6" value="0"/>
  </group_state>
  ```
- **使用示例：**
  ```cpp
  arm.setNamedTarget("zero");
  arm.move();
  ```

```cpp
void setRandomTarget();
```
- **行为：** 随机生成一个满足关节限位且无碰撞的配置
- **用途：** 调试、压力测试
- **注意：** 可能花费较长时间寻找可行配置

```cpp
std::map<std::string, std::vector<double>> getNamedTargets();
```
- **返回：** 所有命名姿态的字典
- **键：** 姿态名称（如 `"zero"`）
- **值：** 关节值向量
- **示例：**
  ```cpp
  auto targets = arm.getNamedTargets();
  for (const auto & [name, joints] : targets) {
    std::cout << name << ": ";
    for (double j : joints) std::cout << j << " ";
    std::cout << std::endl;
  }
  ```

---

#### 1.2.3 笛卡尔位姿目标

```cpp
bool setPoseTarget(const geometry_msgs::msg::Pose & target_pose,
                   const std::string & end_effector_link = "");
```
- **参数：**
  - `target_pose`：目标位姿（在 `getPlanningFrame()` 坐标系下）
  - `end_effector_link`：末端连杆名（空 = `getEndEffectorLink()`）
- **行为：** 设置位置 + 姿态约束（6 自由度）
- **内部：** 创建 `PositionConstraint` 和 `OrientationConstraint`
- **示例：**
  ```cpp
  geometry_msgs::msg::Pose target;
  target.position.x = 0.3;
  target.position.y = 0.0;
  target.position.z = 0.4;
  tf2::Quaternion q;
  q.setRPY(0, M_PI/2, 0);
  target.orientation = tf2::toMsg(q);
  arm.setPoseTarget(target);
  ```

```cpp
bool setPoseTarget(const geometry_msgs::msg::PoseStamped & target_pose,
                   const std::string & end_effector_link = "");
```
- **参数：** 带时间戳和 frame_id 的位姿
- **行为：** 若 `target_pose.header.frame_id != getPlanningFrame()`，自动 TF 变换
- **用途：** 设置在其他坐标系下的目标
- **示例：**
  ```cpp
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "link_tcp";  // 末端坐标系
  target.pose.position.z = 0.1;  // 末端前方 10cm
  target.pose.orientation.w = 1.0;
  arm.setPoseTarget(target);  // 自动转换到 base_link
  ```

```cpp
bool setPoseTargets(const std::vector<geometry_msgs::msg::Pose> & target_poses,
                    const std::string & end_effector_link = "");
```
- **参数：** 多个候选位姿
- **行为：** 规划器会尝试所有位姿，选择代价最低的一个
- **用途：** 增加规划成功率（如对称抓取点）
- **示例：**
  ```cpp
  std::vector<geometry_msgs::msg::Pose> candidates;
  // 两个对称的抓取姿态
  candidates.push_back(pose1);
  candidates.push_back(pose2);
  arm.setPoseTargets(candidates);
  ```

---

#### 1.2.4 仅位置/姿态约束

```cpp
bool setPositionTarget(double x, double y, double z,
                       const std::string & end_effector_link = "");
```
- **参数：** 目标位置（米），在 `getPlanningFrame()` 坐标系下
- **行为：** 只约束位置（3 自由度），姿态自由
- **优势：** IK 成功率高（姿态可任意选择）
- **适用场景：** 点触、抓取（姿态不重要）
- **示例：**
  ```cpp
  arm.setPositionTarget(0.3, 0.0, 0.4);
  ```

```cpp
bool setOrientationTarget(double x, double y, double z, double w,
                          const std::string & end_effector_link = "");
```
- **参数：** 目标四元数
- **行为：** 只约束姿态（3 自由度），位置自由
- **适用场景：** 相机对准、工具角度调整
- **示例：**
  ```cpp
  tf2::Quaternion q;
  q.setRPY(0, M_PI/2, 0);
  arm.setOrientationTarget(q.x(), q.y(), q.z(), q.w());
  ```

```cpp
bool setRPYTarget(double roll, double pitch, double yaw,
                  const std::string & end_effector_link = "");
```
- **参数：** 欧拉角（弧度）
- **行为：** 等价于 `setOrientationTarget()`
- **示例：**
  ```cpp
  arm.setRPYTarget(0, M_PI/2, 0);  // 末端垂直向下
  ```

```cpp
void clearPoseTarget(const std::string & end_effector_link = "");
void clearPoseTargets();
```
- **行为：** 清除之前设置的位姿目标
- **用途：** 切换到关节空间规划前调用

---

### 1.3 规划参数配置

#### 1.3.1 规划器选择

```cpp
void setPlannerId(const std::string & planner_id);
const std::string & getPlannerId() const;
```
- **参数：** 规划器 ID
- **有效值：** 来自 `ompl_planning.yaml` 的 `planner_configs` 键名
- **dm_arm 可用值：**
  - `"RRTConnect"` （默认，快速，适合大多数场景）
  - `"RRTstar"` （最优路径，规划慢）
  - `"PRM"` （多查询场景）
  - `"LBKPIECE"` （窄通道）
  - `"EST"` （Expansive Space Trees）
- **示例：**
  ```cpp
  arm.setPlannerId("RRTstar");
  ```

---

#### 1.3.2 时间与尝试次数

```cpp
void setPlanningTime(double seconds);
double getPlanningTime() const;
```
- **参数：** 单次规划最大允许时间（秒）
- **默认：** 5.0 秒
- **影响：** 超时后返回 `TIMED_OUT`
- **建议：**
  - 简单场景：3.0 秒
  - 复杂场景/窄通道：10.0~15.0 秒
  - 首次调试：15.0 秒

```cpp
void setNumPlanningAttempts(unsigned int num_attempts);
unsigned int getNumPlanningAttempts() const;
```
- **参数：** 规划失败后自动重试次数（含首次）
- **默认：** 1 次（不重试）
- **dm_arm 配置：** 10 次
- **总时间：** 最坏情况 = `num_attempts × planning_time`
- **注意：** 每次重试使用不同的随机种子

---

#### 1.3.3 速度与加速度缩放

```cpp
void setMaxVelocityScalingFactor(double factor);
double getMaxVelocityScalingFactor() const;
```
- **参数：** 速度缩放因子，范围 `(0.0, 1.0]`
- **默认：** 1.0（全速）
- **dm_arm 配置：** 0.3（30% 最大速度）
- **影响：** 时间参数化时，所有关节速度 × factor
- **用途：** 安全限速、调试

```cpp
void setMaxAccelerationScalingFactor(double factor);
double getMaxAccelerationScalingFactor() const;
```
- **参数：** 加速度缩放因子
- **默认：** 1.0
- **dm_arm 配置：** 0.3
- **影响：** 加速/减速段时间增加

---

#### 1.3.4 目标容差

```cpp
void setGoalJointTolerance(double tolerance);
double getGoalJointTolerance() const;
```
- **参数：** 关节空间目标容差（弧度）
- **默认：** 0.0001 rad
- **含义：** 到达目标时，每个关节误差允许 ± tolerance
- **用途：** 放宽容差可提高执行成功率

```cpp
void setGoalPositionTolerance(double tolerance);
double getGoalPositionTolerance() const;
```
- **参数：** 笛卡尔位置目标容差（米）
- **默认：** 0.0001 m
- **dm_arm 配置：** 0.015 m（1.5 cm）
- **用途：** 放宽容差可提高 IK 成功率

```cpp
void setGoalOrientationTolerance(double tolerance);
double getGoalOrientationTolerance() const;
```
- **参数：** 姿态目标容差（弧度）
- **默认：** 0.001 rad（约 0.057°）
- **dm_arm 配置：** 0.05 rad（约 2.9°）
- **含义：** 目标四元数与实际四元数的旋转差 < tolerance

```cpp
void setGoalTolerance(double tolerance);
```
- **参数：** 同时设置位置、姿态、关节容差

---

#### 1.3.5 工作空间限制

```cpp
void setWorkspace(double minx, double miny, double minz,
                  double maxx, double maxy, double maxz);
```
- **参数：** 工作空间的 AABB（轴对齐包围盒）
- **坐标系：** `getPlanningFrame()`
- **行为：** 规划时，末端位置必须在此范围内
- **用途：** 避免规划到危险区域
- **示例：**
  ```cpp
  arm.setWorkspace(-0.8, -0.8, 0.0, 0.8, 0.8, 1.0);
  ```

---

#### 1.3.6 路径约束

```cpp
void setPathConstraints(const moveit_msgs::msg::Constraints & constraint);
moveit_msgs::msg::Constraints getPathConstraints() const;
void clearPathConstraints();
```
- **参数：** 约束消息，包含位置、姿态、可见性等约束
- **行为：** 轨迹上**每个点**都必须满足约束
- **影响：** 显著增加规划时间（10~100 倍）
- **用途：** 工具保持竖直、避免遮挡相机视野
- **示例：**
  ```cpp
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::OrientationConstraint oc;
  oc.link_name = "link_tcp";
  oc.header.frame_id = "base_link";
  oc.orientation.w = 1.0;  // 保持水平
  oc.absolute_x_axis_tolerance = 0.1;
  oc.absolute_y_axis_tolerance = 0.1;
  oc.absolute_z_axis_tolerance = 3.14;  // Z 轴可任意
  oc.weight = 1.0;
  constraints.orientation_constraints.push_back(oc);
  arm.setPathConstraints(constraints);
  ```

---

#### 1.3.7 其他规划参数

```cpp
void allowReplanning(bool flag);
```
- **参数：** 是否允许重规划
- **默认：** false
- **dm_arm 配置：** true
- **行为：** 执行中检测到环境变化时，自动触发重规划
- **用途：** 动态环境下的鲁棒性

```cpp
void allowLooking(bool flag);
```
- **参数：** 是否允许使用传感器数据更新场景
- **默认：** false
- **用途：** 配合深度相机实时更新障碍物

```cpp
void setPlanningPipelineId(const std::string & pipeline_id);
```
- **参数：** 规划管道 ID
- **默认：** `"ompl"`
- **其他值：** `"pilz_industrial_motion_planner"`, `"chomp"`, `"stomp"`

```cpp
void setNumPlanningAttempts(unsigned int num);
```
- **参数：** 规划失败后重试次数
- **默认：** 1

---

### 1.4 规划与执行

#### 1.4.1 规划

```cpp
moveit::core::MoveItErrorCode plan(Plan & plan);
```
- **参数：** 输出参数，存储规划结果
- **返回：** 错误码
- **阻塞：** 是（最长 `planning_time` 秒）
- **副作用：** 不移动机器人
- **Plan 结构：**
  ```cpp
  struct Plan {
    moveit_msgs::msg::RobotState start_state_;
    moveit_msgs::msg::RobotTrajectory trajectory_;
    double planning_time_;
  };
  ```
- **示例：**
  ```cpp
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto error = arm.plan(plan);
  if (error == moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << "Planning time: " << plan.planning_time_ << " sec\n";
    std::cout << "Waypoints: " << plan.trajectory_.joint_trajectory.points.size() << "\n";
  }
  ```

---

#### 1.4.2 执行

```cpp
moveit::core::MoveItErrorCode execute(const Plan & plan);
```
- **参数：** 之前 `plan()` 生成的轨迹
- **返回：** 错误码
- **阻塞：** 是（直到执行完成或超时）
- **超时：** 由 `setExecutionTimeout()` 设置
- **流程：**
  1. 验证轨迹起点与当前状态一致
  2. 查找控制器（通过 `moveit_controllers.yaml`）
  3. 发送 FollowJointTrajectory Action
  4. 等待完成
- **示例：**
  ```cpp
  auto error = arm.execute(plan);
  if (error != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed: %d", error.val);
  }
  ```

```cpp
moveit::core::MoveItErrorCode asyncExecute(const Plan & plan);
```
- **行为：** 非阻塞版本，立即返回
- **查询状态：** `getExecutionInfo()`

---

#### 1.4.3 规划 + 执行

```cpp
moveit::core::MoveItErrorCode move();
```
- **行为：** 等价于 `plan(plan); execute(plan);`
- **阻塞：** 是（规划时间 + 执行时间）
- **用途：** 简化代码
- **示例：**
  ```cpp
  arm.setNamedTarget("zero");
  arm.move();
  ```

```cpp
moveit::core::MoveItErrorCode asyncMove();
```
- **行为：** 非阻塞版本

---

#### 1.4.4 停止

```cpp
void stop();
```
- **行为：** 发送空轨迹给控制器，立即停止运动
- **注意：** 停止后关节位置可能不在预期位置

---

#### 1.4.5 执行超时

```cpp
void setExecutionTimeout(double timeout);
double getExecutionTimeout() const;
```
- **参数：** 执行超时（秒）
- **默认：** 15.0 秒
- **行为：** 若 Action 在 timeout 内未完成，返回 `TIMED_OUT`

---

### 1.5 笛卡尔路径规划

```cpp
double computeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step,
    double jump_threshold,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    bool avoid_collisions = true,
    moveit_msgs::msg::MoveItErrorCodes * error_code = nullptr
);
```

**参数：**
- `waypoints`：路径点序列（在 `getPlanningFrame()` 坐标系下）
- `eef_step`：末端相邻插值点的最大间距（米）
  - 推荐：0.01（1cm）
  - 值越小，轨迹越平滑但点数越多
- `jump_threshold`：关节空间跳跃检测阈值
  - `0.0` = 禁用检测（可能导致关节突变）
  - 推荐：1.5（弧度）
  - 若相邻点的关节差 > threshold，舍弃该路径
- `trajectory`：输出参数，存储笛卡尔轨迹
- `avoid_collisions`：是否检查碰撞
- `error_code`：可选的详细错误信息

**返回：**
- 成功规划的路径比例（0.0 ~ 1.0）
- 1.0 = 完全成功
- 0.5 = 只规划了前 50% 的路径
- 0.0 = 完全失败

**用途：**
- 需要直线运动（如焊接、绘图）
- 避免采样式规划器的随机性

**示例：**
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(start_pose);
waypoints.push_back(mid_pose);
waypoints.push_back(end_pose);

moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = arm.computeCartesianPath(
    waypoints, 0.01, 1.5, trajectory, true);

RCLCPP_INFO(node->get_logger(), "Cartesian path: %.1f%% achieved", fraction * 100.0);

if (fraction > 0.95) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  arm.execute(plan);
}
```

**注意事项：**
1. `computeCartesianPath()` 只生成路径，**没有时间参数化**
2. 需手动调用时间参数化或直接执行（execute 会自动参数化）
3. 若 `fraction < 1.0`，可能因碰撞或 IK 失败
4. 不适合长距离运动（IK 易失败）

---

### 1.6 状态查询

#### 1.6.1 当前状态

```cpp
geometry_msgs::msg::PoseStamped getCurrentPose(
    const std::string & end_effector_link = ""
);
```
- **返回：** 末端当前位姿（带 frame_id 和 timestamp）
- **坐标系：** `getPlanningFrame()`
- **内部：** FK 计算（从 `/joint_states` 获取关节值）
- **阻塞：** 最多 1 秒（等待 joint_states 更新）
- **示例：**
  ```cpp
  auto pose = arm.getCurrentPose();
  std::cout << "Position: (" << pose.pose.position.x << ", "
            << pose.pose.position.y << ", " << pose.pose.position.z << ")\n";
  ```

```cpp
geometry_msgs::msg::Pose getCurrentPose(
    const std::string & end_effector_link = ""
);
```
- **返回：** 不带 header 的版本

```cpp
std::vector<double> getCurrentJointValues();
```
- **返回：** 当前关节值向量
- **顺序：** 与 `getJointNames()` 一致

```cpp
moveit::core::RobotStatePtr getCurrentState(double wait_seconds = 1.0);
```
- **参数：** 等待 joint_states 的超时时间
- **返回：** RobotState 指针（若超时返回 `nullptr`）
- **用途：** 获取完整的机器人状态（用于 FK、IK、碰撞检测）
- **示例：**
  ```cpp
  auto state = arm.getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get current state");
    return;
  }
  ```

```cpp
std::vector<double> getCurrentRPY(
    const std::string & end_effector_link = ""
);
```
- **返回：** 末端当前姿态的欧拉角 `[roll, pitch, yaw]`（弧度）

---

#### 1.6.2 随机状态

```cpp
geometry_msgs::msg::PoseStamped getRandomPose(
    const std::string & end_effector_link = ""
);
```
- **返回：** 随机生成的可达位姿
- **约束：** 满足关节限位且无碰撞
- **用途：** 压力测试、覆盖性验证

---

### 1.7 运动学信息

```cpp
const moveit::core::JointModelGroup* getJointModelGroup() const;
```
- **返回：** JointModelGroup 指针
- **用途：** 访问底层运动学信息（连杆、关节限位、IK 求解器）

```cpp
moveit::core::RobotModelConstPtr getRobotModel() const;
```
- **返回：** RobotModel 指针
- **用途：** 访问完整的机器人模型

```cpp
const std::string & getPoseReferenceFrame() const;
```
- **返回：** 位姿参考坐标系（等同于 `getPlanningFrame()`）

---

### 1.8 记忆的关节值

```cpp
void rememberJointValues(const std::string & name);
```
- **参数：** 记忆名称
- **行为：** 保存当前关节值
- **用途：** 在多步运动中保存中间状态

```cpp
void rememberJointValues(const std::string & name,
                         const std::vector<double> & values);
```
- **参数：** 记忆名称 + 关节值
- **行为：** 保存指定关节值

```cpp
std::vector<double> getRememberedJointValues(const std::string & name);
```
- **返回：** 之前保存的关节值
- **异常：** 若名称不存在，抛出异常

```cpp
void forgetJointValues(const std::string & name);
```
- **行为：** 删除记忆

**示例：**
```cpp
// 记忆当前状态
arm.rememberJointValues("home");

// 移动到其他位置
arm.setRandomTarget();
arm.move();

// 返回记忆的状态
arm.setJointValueTarget(arm.getRememberedJointValues("home"));
arm.move();
```

---

### 1.9 约束采样器

```cpp
void setSupportSurfaceName(const std::string & name);
const std::string & getSupportSurfaceName() const;
```
- **参数：** 支撑表面的碰撞物体 ID
- **用途：** 放置规划（将物体放在桌子上）

---

### 1.10 MoveItErrorCode 完整列表

```cpp
moveit::core::MoveItErrorCode::SUCCESS              //  1
moveit::core::MoveItErrorCode::FAILURE              // -1
moveit::core::MoveItErrorCode::PLANNING_FAILED      // -2
moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN  // -3
moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE  // -4
moveit::core::MoveItErrorCode::CONTROL_FAILED       // -5
moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA  // -6
moveit::core::MoveItErrorCode::TIMED_OUT            // -7
moveit::core::MoveItErrorCode::PREEMPTED            // -8
moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION       // -10
moveit::core::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS  // -11
moveit::core::MoveItErrorCode::GOAL_IN_COLLISION    // -12
moveit::core::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS  // -13
moveit::core::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED       // -14
moveit::core::MoveItErrorCode::INVALID_GROUP_NAME   // -15
moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS        // -16
moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE  // -17
moveit::core::MoveItErrorCode::INVALID_LINK_NAME    // -18
moveit::core::MoveItErrorCode::INVALID_OBJECT_NAME  // -19
moveit::core::MoveItErrorCode::FRAME_TRANSFORM_FAILURE         // -23
moveit::core::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE  // -24
moveit::core::MoveItErrorCode::ROBOT_STATE_STALE    // -25
moveit::core::MoveItErrorCode::SENSOR_INFO_STALE    // -26
moveit::core::MoveItErrorCode::COMMUNICATION_FAILURE// -27
moveit::core::MoveItErrorCode::CRASH                // -29
moveit::core::MoveItErrorCode::ABORT                // -30
moveit::core::MoveItErrorCode::NO_IK_SOLUTION       // -31
```

**常见错误码解读：**

| 错误码 | 含义 | 常见原因 | 解决方法 |
|-------|------|---------|---------|
| -2 | PLANNING_FAILED | 超时/采样失败 | 增加 `planning_time`、`num_planning_attempts` |
| -7 | TIMED_OUT | 规划超时 | 增加 `planning_time` |
| -10 | START_STATE_IN_COLLISION | 起点碰撞 | 检查当前位置，移除障碍物或调整起点 |
| -12 | GOAL_IN_COLLISION | 目标碰撞 | 检查目标位置，调整目标或移除障碍物 |
| -31 | NO_IK_SOLUTION | IK 失败 | 目标超出工作空间或姿态不可达，放宽容差 |

---

## 第二部分：RobotState API

### 2.1 概述

`moveit::core::RobotState` 表示机器人在某一时刻的完整状态，包含所有关节值、连杆变换、速度、力矩等。

**获取方式：**
```cpp
// 从 MoveGroupInterface 获取当前状态
auto state = arm.getCurrentState(2.0);

// 从 RobotModel 创建空白状态
auto model = arm.getRobotModel();
auto state = std::make_shared<moveit::core::RobotState>(model);
```

---

### 2.2 关节值操作

#### 2.2.1 设置关节值

```cpp
void setJointGroupPositions(const std::string & group_name,
                             const std::vector<double> & group_variable_values);
```
- **参数：**
  - `group_name`：规划组名（如 `"arm"`）
  - `group_variable_values`：关节值向量
- **行为：** 设置规划组的关节位置
- **注意：** 不触发 FK 更新，需手动调用 `updateLinkTransforms()`

```cpp
void setJointPositions(const std::string & joint_name, double value);
void setJointPositions(const std::string & joint_name,
                       const std::vector<double> & values);
```
- **参数：** 单个关节的位置值
- **行为：** 设置指定关节

```cpp
void setVariablePositions(const std::vector<double> & position);
```
- **参数：** 所有关节的位置值（按 `getVariableNames()` 顺序）
- **行为：** 设置所有关节

---

#### 2.2.2 获取关节值

```cpp
void copyJointGroupPositions(const std::string & group_name,
                              std::vector<double> & group_variable_values) const;
```
- **参数：** 输出参数，接收关节值
- **行为：** 提取规划组的关节位置
- **示例：**
  ```cpp
  std::vector<double> joints;
  state->copyJointGroupPositions("arm", joints);
  ```

```cpp
const std::vector<double>& getVariablePositions() const;
```
- **返回：** 所有关节位置的 const 引用

```cpp
double getVariablePosition(const std::string & variable) const;
```
- **返回：** 单个关节的位置值

---

### 2.3 正运动学（FK）

```cpp
const Eigen::Isometry3d& getGlobalLinkTransform(const std::string & link_name);
```
- **参数：** 连杆名称
- **返回：** 从 world/base_link 到该连杆的 4×4 变换矩阵
- **注意：** 必须先调用 `updateLinkTransforms()` 刷新 FK
- **示例：**
  ```cpp
  state->setJointGroupPositions("arm", {0, 0, 0, 0, 0, 0});
  state->updateLinkTransforms();
  const Eigen::Isometry3d& tf = state->getGlobalLinkTransform("link_tcp");
  
  Eigen::Vector3d pos = tf.translation();
  Eigen::Quaterniond quat(tf.rotation());
  std::cout << "Position: " << pos.transpose() << "\n";
  std::cout << "Quaternion: " << quat.coeffs().transpose() << "\n";
  ```

```cpp
void updateLinkTransforms();
```
- **行为：** 根据当前关节值，重新计算所有连杆的全局变换
- **用途：** 在设置关节值后调用

```cpp
const Eigen::Isometry3d& getFrameTransform(const std::string & frame_id);
```
- **参数：** 坐标系 ID（可以是连杆名或附加坐标系）
- **返回：** 从 world 到该坐标系的变换

---

### 2.4 逆运动学（IK）

```cpp
bool setFromIK(const moveit::core::JointModelGroup * group,
               const geometry_msgs::msg::Pose & pose,
               double timeout = 0.0,
               const moveit::core::GroupStateValidityCallbackFn & constraint = GroupStateValidityCallbackFn(),
               const kinematics::KinematicsQueryOptions & options = kinematics::KinematicsQueryOptions());
```

**参数：**
- `group`：规划组指针（通过 `arm.getJointModelGroup()` 获取）
- `pose`：目标位姿（在 `getPlanningFrame()` 坐标系下）
- `timeout`：超时时间（秒）
  - `0.0` = 使用 `kinematics.yaml` 中的 `kinematics_solver_timeout`
- `constraint`：可选的状态验证函数（检查关节限位、碰撞）
- `options`：IK 查询选项

**返回：**
- `true`：找到 IK 解，关节值已设置到 `state`
- `false`：无解

**行为：**
1. 调用 IK 求解器（KDL 或 TracIK）
2. 从多个随机种子点出发尝试求解
3. 若成功，更新 `state` 的关节值
4. 调用者需手动调用 `updateLinkTransforms()` 刷新 FK

**示例：**
```cpp
geometry_msgs::msg::Pose target;
target.position.x = 0.3;
target.position.y = 0.0;
target.position.z = 0.4;
target.orientation.w = 1.0;

const auto* jmg = arm.getJointModelGroup();
bool success = state->setFromIK(jmg, target, 0.0);

if (success) {
  std::vector<double> joints;
  state->copyJointGroupPositions("arm", joints);
  std::cout << "IK solution: ";
  for (double j : joints) std::cout << j << " ";
  std::cout << "\n";
} else {
  std::cout << "IK failed\n";
}
```

**带碰撞检测的 IK：**
```cpp
auto validity_callback = [&](moveit::core::RobotState* state,
                              const moveit::core::JointModelGroup* jmg,
                              const double* joint_values) -> bool {
  state->setJointGroupPositions(jmg, joint_values);
  state->updateLinkTransforms();
  return !scene->isStateColliding(*state, jmg->getName());
};

bool success = state->setFromIK(jmg, target, 0.0, validity_callback);
```

---

### 2.5 雅可比矩阵

```cpp
bool getJacobian(const moveit::core::JointModelGroup * group,
                 const moveit::core::LinkModel * link,
                 const Eigen::Vector3d & reference_point_position,
                 Eigen::MatrixXd & jacobian,
                 bool use_quaternion_representation = false) const;
```

**参数：**
- `group`：规划组
- `link`：参考连杆
- `reference_point_position`：参考点在连杆坐标系下的位置（通常为 `{0,0,0}`）
- `jacobian`：输出参数，6×n 矩阵（n = 关节数）
  - 行 0-2：线速度雅可比（∂v/∂q）
  - 行 3-5：角速度雅可比（∂ω/∂q）
- `use_quaternion_representation`：是否用四元数表示姿态

**用途：**
- 速度控制：`v = J * q_dot`
- 奇异性分析：`det(J*J^T) ≈ 0` 表示奇异点
- 力控：`τ = J^T * F`

**示例：**
```cpp
const auto* jmg = arm.getJointModelGroup();
const auto* link = state->getLinkModel("link_tcp");
Eigen::Vector3d ref_point(0, 0, 0);
Eigen::MatrixXd jacobian;

state->getJacobian(jmg, link, ref_point, jacobian);

std::cout << "Jacobian (6x" << jacobian.cols() << "):\n" << jacobian << "\n";

// 奇异性检测
double manipulability = std::sqrt((jacobian * jacobian.transpose()).determinant());
std::cout << "Manipulability: " << manipulability << "\n";
```

---

### 2.6 关节限位检查

```cpp
bool satisfiesBounds(double margin = 0.0) const;
```
- **参数：** 允许的越界裕度（弧度）
- **返回：** 所有关节值是否在限位内
- **示例：**
  ```cpp
  if (!state->satisfiesBounds()) {
    RCLCPP_WARN(node->get_logger(), "State violates joint limits");
  }
  ```

```cpp
bool satisfiesBounds(const moveit::core::JointModelGroup * group,
                     double margin = 0.0) const;
```
- **行为：** 只检查指定规划组

```cpp
void enforceBounds();
```
- **行为：** 将超出限位的关节值截断到限位内
- **示例：**
  ```cpp
  state->setJointGroupPositions("arm", {10.0, 0, 0, 0, 0, 0});  // joint1 超限
  state->enforceBounds();
  // joint1 被截断到 max_position (2.094 rad)
  ```

```cpp
void enforceBounds(const moveit::core::JointModelGroup * group);
```
- **行为：** 只处理指定规划组

```cpp
double distance(const moveit::core::RobotState & other,
                const moveit::core::JointModelGroup * group = nullptr) const;
```
- **参数：** 另一个状态
- **返回：** 关节空间的欧氏距离
- **公式：** `sqrt(Σ(q_i - q_i')^2)`
- **用途：** 判断两个配置是否接近

---

### 2.7 插值

```cpp
void interpolate(const moveit::core::RobotState & to,
                 double t,
                 moveit::core::RobotState & state,
                 const moveit::core::JointModelGroup * group = nullptr) const;
```

**参数：**
- `to`：目标状态
- `t`：插值参数，范围 [0, 1]
  - `t=0`：返回 `this` 状态
  - `t=1`：返回 `to` 状态
  - `t=0.5`：中点
- `state`：输出参数，存储插值结果
- `group`：若指定，只插值该组的关节

**插值公式：**
- 旋转关节（revolute）：线性插值 `q = (1-t)*q_from + t*q_to`
- 连续旋转关节：考虑周期性（如 -π 到 π 的最短路径）

**用途：**
- 生成平滑路径
- 检查中间点是否碰撞

**示例：**
```cpp
auto state1 = arm.getCurrentState();
auto state2 = std::make_shared<moveit::core::RobotState>(*state1);
state2->setJointGroupPositions("arm", {1, 1, 1, 0, 0, 0});

auto mid_state = std::make_shared<moveit::core::RobotState>(*state1);
state1->interpolate(*state2, 0.5, *mid_state);

std::vector<double> mid_joints;
mid_state->copyJointGroupPositions("arm", mid_joints);
std::cout << "Mid-point joints: ";
for (double j : mid_joints) std::cout << j << " ";
std::cout << "\n";
```

---

### 2.8 速度与加速度

```cpp
void setJointGroupVelocities(const std::string & group_name,
                              const std::vector<double> & group_variable_velocities);
void copyJointGroupVelocities(const std::string & group_name,
                               std::vector<double> & group_variable_velocities) const;
```
- **行为：** 设置/获取关节速度

```cpp
void setJointGroupAccelerations(const std::string & group_name,
                                 const std::vector<double> & group_variable_accelerations);
void copyJointGroupAccelerations(const std::string & group_name,
                                  std::vector<double> & group_variable_accelerations) const;
```
- **行为：** 设置/获取关节加速度

---

### 2.9 状态比较

```cpp
bool operator==(const RobotState & other) const;
bool operator!=(const RobotState & other) const;
```
- **行为：** 比较所有关节值（精确比较）

---

## 第三部分：PlanningScene API

### 3.1 概述

`planning_scene::PlanningScene` 是 MoveIt2 的世界模型，包含：
- 机器人当前状态
- 环境障碍物
- 附着物体
- 允许碰撞矩阵（ACM）

**获取方式：**
```cpp
// 通过 PlanningSceneMonitor
planning_scene_monitor::PlanningSceneMonitorPtr monitor = ...;
{
  planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
  // 使用 scene...
}  // 离开作用域自动释放读锁
```

---

### 3.2 碰撞检测

#### 3.2.1 完整碰撞检测

```cpp
void checkCollision(const collision_detection::CollisionRequest & req,
                    collision_detection::CollisionResult & res,
                    const moveit::core::RobotState & state) const;
```

**参数：**
- `req`：碰撞请求配置
- `res`：碰撞结果（输出）
- `state`：待检测的机器人状态

**CollisionRequest 结构：**
```cpp
collision_detection::CollisionRequest req;
req.contacts = true;              // 记录碰撞接触点
req.max_contacts = 5;             // 最多记录 5 个碰撞对
req.max_contacts_per_pair = 1;    // 每对物体最多 1 个接触点
req.distance = false;             // 不计算最小距离（慢）
req.cost = false;                 // 不计算碰撞代价
req.verbose = false;              // 不输出详细日志
```

**CollisionResult 结构：**
```cpp
collision_detection::CollisionResult res;
// 检测后：
res.collision;                    // bool，是否碰撞
res.contact_count;                // 碰撞对数量
res.contacts;                     // map<pair<string,string>, vector<Contact>>
```

**示例：**
```cpp
collision_detection::CollisionRequest req;
req.contacts = true;
req.max_contacts = 10;

collision_detection::CollisionResult res;
scene->checkCollision(req, res, *state);

if (res.collision) {
  RCLCPP_WARN(node->get_logger(), "Collision detected: %zu pairs", res.contacts.size());
  for (const auto & [pair, contacts] : res.contacts) {
    RCLCPP_WARN(node->get_logger(), "  %s <-> %s: %zu contacts",
      pair.first.c_str(), pair.second.c_str(), contacts.size());
    
    for (const auto & contact : contacts) {
      RCLCPP_INFO(node->get_logger(), "    Point: (%.3f, %.3f, %.3f), depth: %.4f",
        contact.pos.x(), contact.pos.y(), contact.pos.z(), contact.depth);
    }
  }
}
```

---

#### 3.2.2 自碰撞检测

```cpp
void checkSelfCollision(const collision_detection::CollisionRequest & req,
                        collision_detection::CollisionResult & res,
                        const moveit::core::RobotState & state) const;
```
- **行为：** 只检查机器人内部连杆之间的碰撞
- **跳过：** SRDF `<disable_collisions>` 中豁免的碰撞对
- **用途：** 验证关节配置合法性

---

#### 3.2.3 快速碰撞检测

```cpp
bool isStateColliding(const moveit::core::RobotState & state,
                      const std::string & group = "",
                      bool verbose = false) const;
```
- **参数：**
  - `state`：待检测状态
  - `group`：若指定，只检查该规划组的连杆
  - `verbose`：是否输出日志
- **返回：** `true` = 碰撞，`false` = 无碰撞
- **优点：** 不记录详细信息，速度快
- **用途：** 规划中的快速碰撞过滤

```cpp
bool isStateValid(const moveit::core::RobotState & state,
                  const std::string & group = "",
                  bool verbose = false) const;
```
- **行为：** 检查碰撞 + 关节限位
- **返回：** `true` = 有效（无碰撞且在限位内）

---

### 3.3 距离查询

```cpp
double distanceToCollision(const moveit::core::RobotState & state) const;
```
- **返回：** 最近障碍物的距离（米）
- **用途：** 安全裕度评估

```cpp
void checkCollision(const collision_detection::CollisionRequest & req,
                    collision_detection::CollisionResult & res,
                    const moveit::core::RobotState & state,
                    const collision_detection::AllowedCollisionMatrix & acm) const;
```
- **参数：** 额外的 ACM（覆盖默认）
- **用途：** 临时允许/禁止某些碰撞对

---

### 3.4 允许碰撞矩阵（ACM）

```cpp
collision_detection::AllowedCollisionMatrix & getAllowedCollisionMatrixNonConst();
const collision_detection::AllowedCollisionMatrix & getAllowedCollisionMatrix() const;
```
- **返回：** ACM 引用
- **用途：** 查询/修改允许碰撞规则

#### 3.4.1 ACM 查询

```cpp
bool getAllowedCollision(const std::string & name1,
                         const std::string & name2,
                         collision_detection::AllowedCollision::Type & type) const;
```
- **参数：** 两个物体名称（连杆或障碍物 ID）
- **返回：** `true` = 规则存在，`false` = 不存在
- **type 可能值：**
  - `ALWAYS`：始终允许碰撞
  - `NEVER`：始终禁止碰撞
  - `CONDITIONAL`：条件允许（如特定距离内）

---

#### 3.4.2 ACM 修改

```cpp
void setEntry(const std::string & name1,
              const std::string & name2,
              bool allowed);
```
- **参数：** 两个物体名称 + 是否允许碰撞
- **示例：**
  ```cpp
  auto & acm = scene->getAllowedCollisionMatrixNonConst();
  
  // 允许夹爪与抓取物体碰撞
  acm.setEntry("finger_left", "target_box", true);
  acm.setEntry("finger_right", "target_box", true);
  
  // 放置后恢复碰撞检测
  acm.removeEntry("finger_left", "target_box");
  acm.removeEntry("finger_right", "target_box");
  ```

```cpp
void removeEntry(const std::string & name1,
                 const std::string & name2);
```
- **行为：** 删除规则（恢复默认行为）

```cpp
void clear();
```
- **行为：** 清空所有规则（不推荐）

```cpp
void print(std::ostream & out) const;
```
- **行为：** 打印 ACM 矩阵到输出流
- **用途：** 调试

---

### 3.5 障碍物管理

#### 3.5.1 添加障碍物

```cpp
void processCollisionObjectMsg(const moveit_msgs::msg::CollisionObject & object);
```
- **参数：** 碰撞物体消息
- **行为：** 根据 `object.operation` 执行 ADD/REMOVE/APPEND/MOVE
- **注意：** 通常不直接调用，而是通过 `PlanningSceneInterface`

---

#### 3.5.2 查询障碍物

```cpp
const collision_detection::WorldPtr & getWorld() const;
```
- **返回：** CollisionWorld 指针
- **用途：** 访问所有障碍物

```cpp
bool hasObjectType(const std::string & object_id,
                   object_recognition_msgs::msg::ObjectType & type) const;
```
- **参数：** 物体 ID
- **返回：** 物体是否存在 + 类型信息

```cpp
std::vector<std::string> getKnownObjectNames(bool with_type = false) const;
```
- **返回：** 所有已知物体 ID

---

### 3.6 附着物体

```cpp
void processAttachedCollisionObjectMsg(
    const moveit_msgs::msg::AttachedCollisionObject & object);
```
- **参数：** 附着物体消息
- **行为：** 将物体附着到机器人连杆

**示例：**
```cpp
// 通过 MoveGroupInterface 附着物体（更简单）
arm.attachObject("target_box", "link_tcp", {"finger_left", "finger_right"});

// 分离物体
arm.detachObject("target_box");
```

---

## 第四部分：TF2 四元数操作

### 4.1 基本转换

#### 4.1.1 RPY → 四元数

```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);  // 单位：弧度
q.normalize();               // 归一化（必须）

// 转换为 ROS 消息
geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
```

**常用姿态速查：**
```cpp
// 水平向前（默认）
q.setRPY(0, 0, 0);

// 垂直向下（末端 Z 轴朝下）
q.setRPY(0, M_PI/2, 0);

// 垂直向上
q.setRPY(0, -M_PI/2, 0);

// 向左侧
q.setRPY(0, 0, M_PI/2);

// 向右侧
q.setRPY(0, 0, -M_PI/2);

// 倒置
q.setRPY(M_PI, 0, 0);
```

---

#### 4.1.2 四元数 → RPY

```cpp
tf2::Quaternion q;
tf2::fromMsg(pose.orientation, q);  // ROS 消息 → tf2

double roll, pitch, yaw;
tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

// 转换为角度
double roll_deg  = roll  * 180.0 / M_PI;
double pitch_deg = pitch * 180.0 / M_PI;
double yaw_deg   = yaw   * 180.0 / M_PI;
```

---

#### 4.1.3 四元数 → 旋转矩阵

```cpp
tf2::Matrix3x3 rot_matrix(q);

// 提取列向量（坐标轴）
tf2::Vector3 x_axis = rot_matrix.getColumn(0);
tf2::Vector3 y_axis = rot_matrix.getColumn(1);
tf2::Vector3 z_axis = rot_matrix.getColumn(2);

std::cout << "X axis: " << x_axis.x() << ", " << x_axis.y() << ", " << x_axis.z() << "\n";
```

---

#### 4.1.4 旋转矩阵 → 四元数

```cpp
tf2::Matrix3x3 rot(
  xx, xy, xz,
  yx, yy, yz,
  zx, zy, zz
);

tf2::Quaternion q;
rot.getRotation(q);
q.normalize();
```

---

### 4.2 四元数运算

#### 4.2.1 四元数乘法（旋转叠加）

```cpp
tf2::Quaternion q1, q2;
q1.setRPY(0, M_PI/2, 0);  // 先绕 Y 轴转 90°
q2.setRPY(0, 0, M_PI/4);  // 再绕 Z 轴转 45°

tf2::Quaternion q_result = q1 * q2;  // 注意顺序：先 q1，后 q2
q_result.normalize();
```

**注意：** 四元数乘法**不满足交换律**！`q1 * q2 ≠ q2 * q1`

**应用场景：**
```cpp
// 在当前姿态基础上旋转
geometry_msgs::msg::Pose current = arm.getCurrentPose().pose;
tf2::Quaternion q_current;
tf2::fromMsg(current.orientation, q_current);

tf2::Quaternion q_delta;
q_delta.setRPY(0, 0, 45.0 * M_PI / 180.0);  // 绕 Z 轴转 45°

tf2::Quaternion q_new = q_current * q_delta;
q_new.normalize();

current.orientation = tf2::toMsg(q_new);
arm.setPoseTarget(current);
```

---

#### 4.2.2 四元数插值（SLERP）

```cpp
tf2::Quaternion q1, q2;
q1.setRPY(0, 0, 0);
q2.setRPY(0, M_PI/2, 0);

// t ∈ [0, 1]
double t = 0.5;
tf2::Quaternion q_mid = q1.slerp(q2, t);
q_mid.normalize();
```

**用途：** 平滑姿态过渡

---

#### 4.2.3 四元数求逆

```cpp
tf2::Quaternion q_inv = q.inverse();
```

**性质：** `q * q_inv = (0, 0, 0, 1)` （单位四元数）

---

### 4.3 坐标系变换

#### 4.3.1 lookupTransform

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer tf_buffer(node->get_clock());
tf2_ros::TransformListener tf_listener(tf_buffer, node);

geometry_msgs::msg::TransformStamped tf =
  tf_buffer.lookupTransform(
    "base_link",          // target_frame
    "link_tcp",           // source_frame
    tf2::TimePointZero,   // 最新时刻
    tf2::durationFromSec(1.0)  // 超时
  );

// tf.transform 包含 translation 和 rotation
std::cout << "Translation: (" << tf.transform.translation.x << ", "
          << tf.transform.translation.y << ", " << tf.transform.translation.z << ")\n";
```

**含义：** 从 `source_frame` 到 `target_frame` 的变换

---

#### 4.3.2 doTransform

```cpp
geometry_msgs::msg::PoseStamped pose_in, pose_out;
pose_in.header.frame_id = "link_tcp";
pose_in.pose.position.z = 0.1;  // 末端前方 10cm
pose_in.pose.orientation.w = 1.0;

geometry_msgs::msg::TransformStamped tf =
  tf_buffer.lookupTransform("base_link", "link_tcp", tf2::TimePointZero);

tf2::doTransform(pose_in, pose_out, tf);

// pose_out 现在在 base_link 坐标系下
std::cout << "Position in base_link: (" << pose_out.pose.position.x << ", "
          << pose_out.pose.position.y << ", " << pose_out.pose.position.z << ")\n";
```

---

### 4.4 前馈方向计算（dm_arm 特例）

**场景：** 让末端 Z 轴指向目标点

```cpp
// 目标点在 base_link 坐标系下
Eigen::Vector3d target_pos(0.3, 0.1, 0.4);

// 末端 Z 轴方向 = 从基座指向目标的单位向量
tf2::Vector3 z_axis(target_pos.x(), target_pos.y(), target_pos.z());
z_axis.normalize();

// 构造正交基
tf2::Vector3 x_axis(1, 0, 0);
if (std::abs(z_axis.dot(x_axis)) > 0.9999) {
  x_axis = tf2::Vector3(0, 1, 0);  // 若 z 与 x 平行，换用 y
}

tf2::Vector3 y_axis = z_axis.cross(x_axis).normalize();
x_axis = y_axis.cross(z_axis);  // 重新计算 x 确保正交

// 构造旋转矩阵（列向量排列）
tf2::Matrix3x3 rot_matrix(
  x_axis.x(), y_axis.x(), z_axis.x(),
  x_axis.y(), y_axis.y(), z_axis.y(),
  x_axis.z(), y_axis.z(), z_axis.z()
);

tf2::Quaternion q_feedforward;
rot_matrix.getRotation(q_feedforward);
q_feedforward.normalize();

// 应用到目标位姿
geometry_msgs::msg::Pose target;
target.position.x = target_pos.x();
target.position.y = target_pos.y();
target.position.z = target_pos.z();
target.orientation = tf2::toMsg(q_feedforward);

arm.setPoseTarget(target);
```

---

## 第五部分：ros2_control 生命周期

### 5.1 状态机图

```
                  ┌──────────────┐
                  │  UNCONFIGURED│ ← 初始状态
                  └──────┬───────┘
                         │ on_configure()
                         ↓
                  ┌──────────────┐
                  │   INACTIVE   │ ← 配置完成，未激活
                  └──┬────────┬──┘
            on_activate│        │on_cleanup()
                       ↓        ↓
                  ┌──────────────┐    ┌──────────────┐
                  │    ACTIVE    │    │ UNCONFIGURED │
                  └──┬───────────┘    └──────────────┘
            on_deactivate│
                         ↓
                  ┌──────────────┐
                  │   INACTIVE   │
                  └──────────────┘
                  
                任意状态 ──on_error()──→ ERROR
```

---

### 5.2 生命周期回调详解

#### 5.2.1 on_init()

**调用时机：** 插件加载后立即调用（一次）

**职责：**
1. 从 `HardwareInfo` 解析参数
2. 验证配置合法性
3. 分配内存（`hw_positions_`, `hw_commands_`）
4. **不做耗时操作**（如打开串口）

**返回值：**
- `SUCCESS`：继续初始化
- `ERROR`：中止，节点崩溃

**示例：**
```cpp
hardware_interface::CallbackReturn DmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 解析参数
  serial_port_ = info_.hardware_parameters.at("serial_port");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

  // 分配缓冲区
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

---

#### 5.2.2 on_configure()

**调用时机：** 执行 `ros2 control load_controller` 后

**职责：**
1. 打开串口
2. 创建电机对象
3. **不使能电机**（留给 `on_activate`）

**返回值：**
- `SUCCESS` → 进入 INACTIVE 状态
- `ERROR` → 进入 ERROR 状态

**示例：**
```cpp
hardware_interface::CallbackReturn DmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    serial_ = std::make_shared<SerialPort>(serial_port_, baudrate_);
    motor_controller_ = std::make_shared<damiao::Motor_Control>(serial_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
      "Failed to open serial: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 创建电机对象（不使能）
  for (const auto & cfg : joint_configs_) {
    auto motor = std::make_shared<damiao::Motor>(...);
    motors_.push_back(motor);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

---

#### 5.2.3 on_activate()

**调用时机：** 执行 `ros2 control set_controller_state <name> activate` 后

**职责：**
1. 使能电机
2. 读取初始位置
3. 设置初始命令值
4. **开始控制循环**

**返回值：**
- `SUCCESS` → 进入 ACTIVE 状态，开始调用 `read()`/`write()`
- `ERROR` → 进入 ERROR 状态

**示例：**
```cpp
hardware_interface::CallbackReturn DmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 使能电机
  for (auto & motor : motors_) {
    motor_controller_->enable(*motor);
  }

  // 读取初始位置（多次平均）
  for (int i = 0; i < 5; ++i) {
    read(rclcpp::Time{}, rclcpp::Duration{0, 0});
    usleep(20 * 1000);
  }

  // 设置初始命令 = 当前位置
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];
    hw_commands_prev_[i] = hw_positions_[i];
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

---

#### 5.2.4 on_deactivate()

**调用时机：** 执行 `ros2 control set_controller_state <name> deactivate` 后

**职责：**
1. 平滑归零（避免突然断电）
2. 失能电机
3. **停止控制循环**

**返回值：**
- `SUCCESS` → 进入 INACTIVE 状态，停止调用 `read()`/`write()`

**示例：**
```cpp
hardware_interface::CallbackReturn DmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return_zero_smooth();  // 平滑归零

  // 失能电机
  for (auto & motor : motors_) {
    motor_controller_->disable(*motor);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
```

---

#### 5.2.5 on_cleanup()

**调用时机：** 节点关闭前

**职责：**
1. 释放资源（串口、内存）
2. 清理临时文件

**返回值：**
- `SUCCESS` → 进入 UNCONFIGURED 状态

**示例：**
```cpp
hardware_interface::CallbackReturn DmHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motors_.clear();
  motor_controller_.reset();
  serial_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}
```

---

#### 5.2.6 on_error()

**调用时机：** 任意回调返回 ERROR 时

**职责：** 记录错误信息，尝试恢复

**返回值：**
- `SUCCESS` → 可重新 configure
- `FAILURE` → 无法恢复，节点崩溃

---

### 5.3 read() 与 write()

#### 5.3.1 read()

**调用频率：** 由 `control_frequency` 决定（dm_arm: 500Hz）

**职责：** 从硬件读取状态，更新 `hw_positions_`, `hw_velocities_`, `hw_efforts_`

**签名：**
```cpp
hardware_interface::return_type read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period
);
```

**参数：**
- `time`：当前时刻（绝对时间）
- `period`：距离上次 `read()` 的时间间隔

**返回值：**
- `OK`：成功
- `ERROR`：硬件故障

---

#### 5.3.2 write()

**调用频率：** 500Hz（在 `read()` 之后立即调用）

**职责：** 将 `hw_commands_` 下发到硬件

**签名：**
```cpp
hardware_interface::return_type write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period
);
```

**数据流：**
```
控制器 → hw_commands_[i] → write() → 电机驱动 → 硬件
硬件 → 电机反馈 → read() → hw_positions_[i] → 控制器
```

---

## 第六部分：常见报错与解决

### 6.1 规划失败

#### 6.1.1 PLANNING_FAILED (-2)

**症状：**
```
[ERROR] Planning failed with error code: -2
```

**原因：**
- 规划超时
- 采样空间受限（障碍物过多）
- 起点或终点在奇异点附近

**解决方法：**
1. 增加规划时间：
   ```cpp
   arm.setPlanningTime(15.0);
   ```
2. 增加重试次数：
   ```cpp
   arm.setNumPlanningAttempts(20);
   ```
3. 更换规划器：
   ```cpp
   arm.setPlannerId("RRTstar");
   ```
4. 检查障碍物：
   ```bash
   ros2 topic echo /planning_scene
   ```

---

#### 6.1.2 NO_IK_SOLUTION (-31)

**症状：**
```
[ERROR] Planning failed with error code: -31
```

**原因：**
- 目标位置超出工作空间
- 目标姿态不可达
- IK 求解器超时

**解决方法：**
1. 验证目标距离：
   ```cpp
   double distance = std::sqrt(x*x + y*y + z*z);
   if (distance < 0.1 || distance > 0.6) {
     RCLCPP_WARN(node->get_logger(), "Target out of workspace");
   }
   ```
2. 放宽姿态约束：
   ```cpp
   arm.setGoalOrientationTolerance(0.2);  // 约 11.5°
   ```
3. 使用仅位置目标：
   ```cpp
   arm.setPositionTarget(x, y, z);  // 姿态自由
   ```
4. 增加 IK 超时：
   ```yaml
   # config/kinematics.yaml
   arm:
     kinematics_solver_timeout: 0.05  # 增加到 50ms
     kinematics_solver_attempts: 5
   ```
5. 更换 IK 求解器（TracIK 成功率更高）：
   ```bash
   sudo apt install ros-humble-trac-ik-kinematics-plugin
   ```
   ```yaml
   # config/kinematics.yaml
   arm:
     kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
   ```

---

#### 6.1.3 START_STATE_IN_COLLISION (-10)

**症状：**
```
[ERROR] Planning failed: Start state in collision
```

**原因：** 当前机器人状态与障碍物碰撞

**解决方法：**
1. 检查当前状态：
   ```bash
   ros2 topic echo /joint_states
   ```
2. 移除多余障碍物：
   ```cpp
   scene.removeCollisionObjects(scene.getKnownObjectNames());
   ```
3. 手动微调关节值：
   ```cpp
   auto state = arm.getCurrentState();
   state->enforceBounds();  // 截断到限位内
   std::vector<double> joints;
   state->copyJointGroupPositions("arm", joints);
   arm.setJointValueTarget(joints);
   arm.move();
   ```

---

#### 6.1.4 GOAL_IN_COLLISION (-12)

**症状：**
```
[ERROR] Goal state in collision
```

**原因：** 目标位姿与障碍物碰撞

**解决方法：**
1. 可视化障碍物（RViz）
2. 调整目标位置
3. 临时允许碰撞：
   ```cpp
   auto & acm = scene->getAllowedCollisionMatrixNonConst();
   acm.setEntry("link_tcp", "obstacle1", true);
   ```

---

### 6.2 执行失败

#### 6.2.1 CONTROL_FAILED (-5)

**症状：**
```
[ERROR] Execution failed with error code: -5
```

**原因：**
- 控制器未激活
- 硬件接口错误
- 轨迹跟踪超时

**解决方法：**
1. 检查控制器状态：
   ```bash
   ros2 control list_controllers
   # 预期：arm_controller [active]
   ```
2. 手动激活：
   ```bash
   ros2 control set_controller_state arm_controller activate
   ```
3. 检查 joint_states 频率：
   ```bash
   ros2 topic hz /joint_states
   # 预期：50Hz 以上
   ```
4. 检查硬件接口日志：
   ```bash
   ros2 run dm_arm_hardware ...
   ```

---

#### 6.2.2 TIMED_OUT (-7)

**症状：**
```
[ERROR] Execution timed out
```

**原因：**
- 轨迹执行时间超过 `execution_timeout`
- 硬件响应慢

**解决方法：**
1. 增加超时：
   ```cpp
   arm.setExecutionTimeout(30.0);
   ```
2. 降低速度缩放：
   ```cpp
   arm.setMaxVelocityScalingFactor(0.1);  // 降到 10%
   ```

---

### 6.3 硬件接口错误

#### 6.3.1 Failed to load hardware interface

**症状：**
```
[ERROR] Failed to load class 'dm_arm_hardware/DmHardwareInterface'
```

**原因：**
- pluginlib 未找到插件
- 插件描述文件路径错误

**解决方法：**
1. 检查插件是否安装：
   ```bash
   ros2 pkg list | grep dm_arm_hardware
   ```
2. 检查插件描述文件：
   ```bash
   cat install/dm_arm_hardware/share/dm_arm_hardware/dm_arm_hardware.xml
   ```
3. 验证 `package.xml` 导出：
   ```xml
   <export>
     <hardware_interface plugin="${prefix}/dm_arm_hardware.xml"/>
   </export>
   ```

---

#### 6.3.2 StateInterface pointer invalidated

**症状：**
```
[ERROR] Segmentation fault when accessing hw_positions_[i]
```

**原因：** `hw_positions_` 向量重新分配，指针失效

**解决方法：**
- **禁止** 在 `on_init()` 之后调用 `push_back()`/`resize()`
- 只在 `on_init()` 中分配一次：
  ```cpp
  hw_positions_.assign(n, 0.0);  // ✓
  hw_positions_.resize(n);       // ✓
  hw_positions_.push_back(0.0);  // ✗ 禁止！
  ```

---

### 6.4 TF 相关错误

#### 6.4.1 tf2::LookupException

**症状：**
```
[ERROR] "link_tcp" passed to lookupTransform argument target_frame does not exist.
```

**原因：** TF 树不完整

**解决方法：**
1. 检查 TF 树：
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```
2. 确认 robot_state_publisher 运行：
   ```bash
   ros2 node list | grep robot_state_publisher
   ```
3. 检查 URDF 加载：
   ```bash
   ros2 param get /robot_state_publisher robot_description
   ```

---

### 6.5 参数错误

#### 6.5.1 Invalid parameter type

**症状：**
```
[ERROR] parameter 'joint_limits.joint1.max_velocity' has invalid type:
expected [double] got [integer]
```

**原因：** YAML 中整数应写为浮点数

**解决方法：**
```yaml
# 错误
max_velocity: 3

# 正确
max_velocity: 3.0
```

---

#### 6.5.2 Undeclared parameter

**症状：**
```
[ERROR] Parameter 'moveit.planning_time' has not been declared
```

**原因：** ROS2 必须先 `declare_parameter()` 才能 `get_parameter()`

**解决方法：**
```cpp
node->declare_parameter("moveit.planning_time", 5.0);
double t = node->get_parameter("moveit.planning_time").as_double();
```

---

## 附录：快速参考表

### A.1 MoveGroupInterface 常用方法

| 方法 | 功能 | 常用参数 |
|-----|------|---------|
| `setJointValueTarget(vec)` | 设置关节目标 | 6 个关节值（弧度） |
| `setNamedTarget(name)` | 使用命名姿态 | `"zero"`, `"home"` |
| `setPoseTarget(pose)` | 设置位姿目标 | 位置 + 四元数 |
| `setPositionTarget(x,y,z)` | 仅位置约束 | 米 |
| `plan(plan)` | 规划轨迹 | 输出 Plan 结构 |
| `execute(plan)` | 执行轨迹 | 阻塞直到完成 |
| `move()` | 规划+执行 | 等价于 plan+execute |
| `getCurrentPose()` | 获取末端位姿 | 返回 PoseStamped |
| `getCurrentJointValues()` | 获取关节值 | 返回 vector<double> |

---

### A.2 错误码速查

| 错误码 | 含义 | 快速解决 |
|-------|------|---------|
| 1 | SUCCESS | - |
| -2 | PLANNING_FAILED | ↑ planning_time |
| -7 | TIMED_OUT | ↑ planning_time |
| -10 | START_STATE_IN_COLLISION | 移除障碍物 |
| -12 | GOAL_IN_COLLISION | 调整目标 |
| -31 | NO_IK_SOLUTION | ↑ tolerance 或换 TracIK |

---

### A.3 配置文件速查

| 文件 | 关键参数 | 默认值 |
|-----|---------|--------|
| `kinematics.yaml` | `kinematics_solver_timeout` | 0.005 秒 |
| `ompl_planning.yaml` | `longest_valid_segment_fraction` | 0.005 |
| `joint_limits.yaml` | `max_velocity` | 3.0 rad/s |
| `ros2_controllers.yaml` | `update_rate` | 500 Hz |

---

**文档结束**

如需更多信息，请参阅：
- [MoveIt2 官方文档](https://moveit.picknik.ai/main/index.html)
- [ros2_control 官方文档](https://control.ros.org)
