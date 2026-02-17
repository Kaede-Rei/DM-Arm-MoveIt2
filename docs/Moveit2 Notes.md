# dm_arm MoveIt2 详细 API 笔记

> 以 dm_arm 项目为基准，补充教程中未展开的 API 文档、行为细节与注意事项。

---

## 一、MoveGroupInterface 完整 API

### 1.1 构造与初始化

```cpp
// ROS2 构造签名（位于 moveit/move_group_interface/move_group_interface.h）
moveit::planning_interface::MoveGroupInterface(
    const rclcpp::Node::SharedPtr & node,
    const std::string & group,                          // SRDF 中的规划组名
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer  // 可选，若不传则内部创建
        = std::shared_ptr<tf2_ros::Buffer>(),
    const rclcpp::Duration & wait_for_servers            // 等待 move_group 动作服务器超时
        = rclcpp::Duration::from_seconds(0)              // 0 = 无限等待
);
```

**注意事项：**
- 构造函数会**阻塞**直到 `/move_group` 动作服务器可达，因此 `move_group` 节点必须先于 dm_arm_server 启动。教程中的 `TimerAction(period=2.0)` 就是为此而设。
- 若 `wait_for_servers` 超时，构造函数会抛出异常而非返回错误，必须用 `try-catch` 包裹。
- 不要在构造函数外部、节点 spin 之前调用任何 API，因为 TF 和 joint_states 的订阅需要 executor 在运行。

---

### 1.2 目标设置 API

```cpp
// ── 关节空间目标 ──────────────────────────────────────────────────────────
bool setJointValueTarget(const std::vector<double> & group_variable_values);
// 参数顺序与 getJointNames() 返回顺序一致
// 对 dm_arm 为：[joint1, joint2, joint3, joint4, joint5, joint6]

bool setJointValueTarget(const std::string & joint_name, double value);
// 设置单个关节，其余关节保持当前值

bool setJointValueTarget(const moveit::core::RobotState & robot_state);
// 以 RobotState 作为目标

bool setJointValueTarget(const sensor_msgs::msg::JointState & state);
// 以 JointState 消息作为目标，忽略消息中不属于规划组的关节

// ── 命名目标（来自 SRDF group_state） ────────────────────────────────────
bool setNamedTarget(const std::string & name);
// dm_arm 使用：setNamedTarget("zero") → SRDF 中 group_state name="zero"
// 若 name 不存在则返回 false，不抛异常

// ── 笛卡尔位姿目标 ────────────────────────────────────────────────────────
bool setPoseTarget(
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & end_effector_link = "");
// end_effector_link 为空时使用 getEndEffectorLink() 的值
// dm_arm 末端：link_tcp

bool setPoseTarget(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const std::string & end_effector_link = "");
// 带坐标系的版本，MoveIt 会自动做坐标变换

bool setPoseTargets(
    const std::vector<geometry_msgs::msg::Pose> & target_poses,
    const std::string & end_effector_link = "");
// 设置多个候选目标，规划器选择最优一个

// ── 仅位置目标（忽略姿态） ────────────────────────────────────────────────
bool setPositionTarget(
    double x, double y, double z,
    const std::string & end_effector_link = "");
// 只约束位置，姿态自由，IK 成功率高

// ── 仅姿态目标（忽略位置） ────────────────────────────────────────────────
bool setOrientationTarget(
    double x, double y, double z, double w,
    const std::string & end_effector_link = "");

// ── 随机目标（调试用） ────────────────────────────────────────────────────
bool setRandomTarget();

// ── 清除目标 ─────────────────────────────────────────────────────────────
void clearPoseTarget(const std::string & end_effector_link = "");
void clearPoseTargets();  // 清除所有末端的目标
```

**dm_arm 的实际用法对应：**

| 服务命令 | MoveGroupInterface 调用 |
|---------|------------------------|
| `zero` | `setNamedTarget("zero")` → `move()` |
| `goal_base` | `setPoseTarget(target_pose)` → `plan()` → `execute(plan)` |
| `goal_eef` | 先 TF 变换 → 同 goal_base |
| `stretch` | 构造末端坐标系偏移 → `setGoalPoseEef` |
| `rotate` | 计算新四元数 → `setPoseTarget` |

---

### 1.3 规划参数 API

```cpp
// ── 规划器选择 ────────────────────────────────────────────────────────────
void setPlannerId(const std::string & planner_id);
// 有效值来自 ompl_planning.yaml 的 planner_configs 键名
// dm_arm 默认：RRTConnect
// 其他常用：RRTstar、PRM、LazyPRM、LBKPIECE、BiEST

// ── 时间与尝试次数 ────────────────────────────────────────────────────────
void setPlanningTime(double seconds);
// 单次规划最大允许时间，超时返回失败
// dm_arm 配置：5.0 秒
// 建议：首次运行可设 10.0，稳定后降至 3.0

void setNumPlanningAttempts(unsigned int num_attempts);
// 规划失败后自动重试次数（含首次）
// dm_arm 配置：10 次
// 注意：总时间 = num_attempts × planning_time（最坏情况）

// ── 重规划 ────────────────────────────────────────────────────────────────
void allowReplanning(bool flag);
// true：若执行中检测到环境变化（障碍物出现），自动触发重规划
// false：执行失败直接报错
// dm_arm 配置：true

// ── 速度与加速度限制 ──────────────────────────────────────────────────────
void setMaxVelocityScalingFactor(double factor);
// 范围：(0.0, 1.0]，1.0 = joint_limits.yaml 中定义的最大速度
// dm_arm 配置：0.3（30% 最大速度）
// 注意：此参数影响时间参数化，不影响路径形状

void setMaxAccelerationScalingFactor(double factor);
// 同上，影响加速/减速段时间
// dm_arm 配置：0.3

// ── 目标容差 ─────────────────────────────────────────────────────────────
void setGoalJointTolerance(double tolerance);
// 关节空间目标的到达容差（弧度）
// 默认：0.0001 rad

void setGoalPositionTolerance(double tolerance);
// 笛卡尔位置目标的到达容差（米）
// dm_arm 配置：0.015 m（1.5cm）

void setGoalOrientationTolerance(double tolerance);
// 笛卡尔姿态目标的到达容差（弧度）
// dm_arm 配置：0.05 rad

void setGoalTolerance(double tolerance);
// 同时设置位置、姿态和关节容差（用同一个值）

// ── 路径约束 ─────────────────────────────────────────────────────────────
void setPathConstraints(const moveit_msgs::msg::Constraints & constraint);
// 规划时整条轨迹上每个点都必须满足该约束
// 会显著增加规划时间，建议 planning_time 设为 15.0+

void clearPathConstraints();
moveit_msgs::msg::Constraints getPathConstraints() const;

// ── 工作空间限制 ──────────────────────────────────────────────────────────
void setWorkspace(
    double minx, double miny, double minz,
    double maxx, double maxy, double maxz);
// 限制规划时采样点的工作空间范围，超出范围的配置不参与规划
// dm_arm 参考：setWorkspace(-0.8,-0.8,0.0, 0.8,0.8,1.0)

// ── 起始状态 ─────────────────────────────────────────────────────────────
void setStartStateToCurrentState();
// 将规划起点设为当前关节状态
// 每次规划前都应调用，否则使用上次规划的终点作为起点

void setStartState(const moveit::core::RobotState & start_state);
// 手动指定起始状态（用于离线规划）

void setStartState(const moveit_msgs::msg::RobotState & start_state);
```

---

### 1.4 规划与执行 API

```cpp
// ── plan()：仅规划，不执行 ───────────────────────────────────────────────
moveit::core::MoveItErrorCode plan(
    moveit::planning_interface::MoveGroupInterface::Plan & plan);

// Plan 结构体：
struct Plan {
    moveit_msgs::msg::RobotState start_state_;        // 起始状态
    moveit_msgs::msg::RobotTrajectory trajectory_;    // 关节轨迹（主要数据）
    double planning_time_;                            // 规划耗时（秒）
};

// trajectory_.joint_trajectory.points 是轨迹点数组
// 每个点：positions（关节角）、velocities、accelerations、time_from_start

// MoveItErrorCode 常用值：
// moveit::core::MoveItErrorCode::SUCCESS
// moveit::core::MoveItErrorCode::PLANNING_FAILED
// moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN
// moveit::core::MoveItErrorCode::TIMED_OUT
// moveit::core::MoveItErrorCode::NO_IK_SOLUTION
// moveit::core::MoveItErrorCode::GOAL_IN_COLLISION
// moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION

// 用法：
moveit::planning_interface::MoveGroupInterface::Plan plan;
auto error_code = arm_.plan(plan);
if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    // error_code.val 是 int32，可直接打印
    RCLCPP_ERROR(node_->get_logger(), "Plan failed: %d", error_code.val);
}

// ── execute()：执行已规划轨迹 ─────────────────────────────────────────────
moveit::core::MoveItErrorCode execute(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);
// 同步执行，阻塞直到完成或超时

// ── asyncExecute()：异步执行 ──────────────────────────────────────────────
moveit::core::MoveItErrorCode asyncExecute(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan);
// 立即返回，通过 getExecutionInfo() 查询状态

// ── move()：规划并执行的组合 ──────────────────────────────────────────────
moveit::core::MoveItErrorCode move();
// 内部调用 plan() 然后 execute()
// dm_arm 的 resetToZero() 使用此接口

// ── asyncMove()：异步规划并执行 ───────────────────────────────────────────
moveit::core::MoveItErrorCode asyncMove();

// ── stop()：停止当前运动 ──────────────────────────────────────────────────
void stop();
// 发送空轨迹给控制器，立即停止
// 注意：停止后机器人状态可能不在预期位置

// ── 笛卡尔路径规划（直线插值） ────────────────────────────────────────────
double computeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step,                    // 末端相邻插值点间距（米），建议 0.01
    double jump_threshold,              // 关节空间跳跃阈值，0.0 = 禁用跳跃检测
    moveit_msgs::msg::RobotTrajectory & trajectory,
    bool avoid_collisions = true);
// 返回值：成功规划的比例（0.0~1.0），1.0 = 完全成功
// 注意：jump_threshold=0 在实际场景中容易导致关节突变，建议设 1.5
// 用于需要直线轨迹的场景（如搬运过程中的平移）
```

---

### 1.5 状态查询 API

```cpp
// ── 当前末端位姿 ──────────────────────────────────────────────────────────
geometry_msgs::msg::PoseStamped getCurrentPose(
    const std::string & end_effector_link = "");
// 返回末端在 getPlanningFrame() 坐标系下的位姿（带时间戳和 frame_id）
// dm_arm 使用：arm_.getCurrentPose().pose

// ── 当前关节值 ────────────────────────────────────────────────────────────
std::vector<double> getCurrentJointValues();
// 返回规划组内所有关节的当前位置，顺序与 getJointNames() 一致

// ── 当前 RobotState ───────────────────────────────────────────────────────
moveit::core::RobotStatePtr getCurrentState(double wait_seconds = 1.0);
// ROS2 版本：需指定超时秒数，建议 2.0
// 返回 nullptr 表示超时（如 joint_states 未发布）
// 获取后可用于：IK计算、碰撞检测、FK计算、关节限位检查

// ── 随机位姿（调试用） ────────────────────────────────────────────────────
geometry_msgs::msg::PoseStamped getRandomPose(
    const std::string & end_effector_link = "");

// ── 组信息 ────────────────────────────────────────────────────────────────
const std::string & getName() const;
// 返回规划组名称，dm_arm 返回 "arm"

const std::string & getPlanningFrame() const;
// 返回规划参考系，通常是 "base_link" 或 "world"
// 从 SRDF + URDF 中解析，dm_arm 返回 "base_link"

const std::string & getEndEffectorLink() const;
// 返回末端连杆名，由 SRDF end_effector 或 chain tip_link 决定
// dm_arm 返回 "link_tcp"（chain tip）

const std::vector<std::string> & getJointNames();
// 规划组内所有关节名称（按 SRDF chain 顺序）
// dm_arm arm 组：[joint1, joint2, joint3, joint4, joint5, joint6]

const std::vector<std::string> & getLinkNames();
// 规划组内所有连杆名称（按运动学链顺序）
// dm_arm arm 组：[base_link, link1-2, link1, link2, link4-5, link5-6, link6-7, link_tcp]

double getGoalPositionTolerance() const;
double getGoalOrientationTolerance() const;
double getGoalJointTolerance() const;
```

---

## 二、RobotState 完整 API

```cpp
// 获取 RobotState（通常从 MoveGroupInterface 获取）
moveit::core::RobotStatePtr state = arm_.getCurrentState(2.0);

// ── 正运动学（FK） ────────────────────────────────────────────────────────
const Eigen::Isometry3d & getGlobalLinkTransform(const std::string & link_name);
// 返回 link 在全局（world/base_link）坐标系下的变换矩阵
// 需要先调用 setJointGroupPositions() 设置关节值

// 示例：获取 link_tcp 的全局位姿
state->setJointGroupPositions("arm", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
state->updateLinkTransforms();  // 必须调用以刷新 FK
const Eigen::Isometry3d & tf = state->getGlobalLinkTransform("link_tcp");
Eigen::Vector3d pos = tf.translation();
Eigen::Quaterniond quat(tf.rotation());

// ── 逆运动学（IK） ────────────────────────────────────────────────────────
bool setFromIK(
    const moveit::core::JointModelGroup * group,
    const geometry_msgs::msg::Pose & pose,
    double timeout = 0.0,                           // 0 = 使用 kinematics.yaml 配置值
    const moveit::core::GroupStateValidityCallbackFn & constraint = {});
// 返回 true：找到 IK 解，并已设置到 state 的关节值中
// 返回 false：无解
// 注意：找到解后 state 的关节值已被修改，可直接用 state->copyJointGroupPositions() 获取

// 验证后获取关节值：
std::vector<double> joint_values;
state->copyJointGroupPositions("arm", joint_values);

// ── 关节值操作 ────────────────────────────────────────────────────────────
void setJointGroupPositions(
    const std::string & joint_group_name,
    const std::vector<double> & gstate);
// 设置规划组的关节值（不触发 FK 更新）

void copyJointGroupPositions(
    const std::string & joint_group_name,
    std::vector<double> & gstate) const;
// 获取规划组的关节值到向量

// ── 关节限位检查 ──────────────────────────────────────────────────────────
bool satisfiesBounds(double margin = 0.0) const;
// margin > 0：允许超出限位 margin 弧度
// 用于在设置目标前快速检查关节值是否合法

bool satisfiesBounds(
    const moveit::core::JointModelGroup * group,
    double margin = 0.0) const;
// 只检查指定规划组

void enforceBounds();
// 将超出限位的关节值截断到限位内
void enforceBounds(const moveit::core::JointModelGroup * group);

// ── 雅可比矩阵 ────────────────────────────────────────────────────────────
bool getJacobian(
    const moveit::core::JointModelGroup * group,
    const moveit::core::LinkModel * link,
    const Eigen::Vector3d & reference_point_position,
    Eigen::MatrixXd & jacobian,
    bool use_quaternion_representation = false) const;
// 用于速度控制和奇异性分析
// reference_point_position：参考点相对于 link 坐标系的偏移，通常为 {0,0,0}

// ── 碰撞检测（需要 PlanningScene） ───────────────────────────────────────
// 见 PlanningScene 章节

// ── 距离与插值 ────────────────────────────────────────────────────────────
double distance(
    const moveit::core::RobotState & other,
    const moveit::core::JointModelGroup * group) const;
// 计算两个状态在关节空间的距离（欧氏范数）

void interpolate(
    const moveit::core::RobotState & to,
    double t,
    moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * group) const;
// 在 this 和 to 之间以比例 t (0~1) 插值，结果写入 state
```

---

## 三、PlanningSceneMonitor 与 PlanningScene API

### 3.1 PlanningSceneMonitor

```cpp
// 构造（ROS2 必须传 node）
planning_scene_monitor::PlanningSceneMonitorPtr monitor =
    std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node, "robot_description");  // 第二参数：robot_description 参数名

// ── 启动监控器（各监控器独立启动） ──────────────────────────────────────
monitor->startSceneMonitor("/planning_scene");
// 订阅 /planning_scene 话题，接收 MoveIt 场景更新

monitor->startWorldGeometryMonitor();
// 订阅 /collision_object、/attached_collision_object
// 接收外部发布的碰撞物体更新

monitor->startStateMonitor("/joint_states");
// 订阅 /joint_states，跟踪关节状态实时变化
// 必须启动，否则 getCurrentState() 可能返回 nullptr

// ── 获取 PlanningScene 对象 ───────────────────────────────────────────────
// 方式一：带读锁（不可修改）
{
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    // scene 对象在此块内有效，离开块自动解锁
    bool colliding = scene->isStateColliding(*state, "arm");
}

// 方式二：带写锁（可修改）
{
    planning_scene_monitor::LockedPlanningSceneRW scene(monitor);
    scene->getAllowedCollisionMatrixNonConst().setEntry("link1", "link2", true);
}
// LockedPlanningSceneRW 用完后必须调用 monitor->triggerSceneUpdateEvent() 通知其他监听者
monitor->triggerSceneUpdateEvent(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
```

### 3.2 PlanningScene 碰撞检测

```cpp
// ── 检查指定状态下的碰撞 ─────────────────────────────────────────────────
collision_detection::CollisionRequest req;
req.contacts = true;              // 记录碰撞接触点
req.max_contacts = 5;             // 最多记录 5 个碰撞对
req.max_contacts_per_pair = 1;    // 每对物体最多 1 个接触点
req.distance = false;             // 不计算最小距离（计算慢）
req.verbose = false;

collision_detection::CollisionResult res;
scene->checkCollision(req, res, *state);

if (res.collision) {
    RCLCPP_WARN(logger, "Collision detected: %zu pairs", res.contacts.size());
    for (const auto & [pair, contacts] : res.contacts) {
        RCLCPP_WARN(logger, "  %s <-> %s (%zu points)",
            pair.first.c_str(), pair.second.c_str(), contacts.size());
    }
}

// ── 自碰撞检查 ────────────────────────────────────────────────────────────
scene->checkSelfCollision(req, res, *state);

// ── 快速碰撞检查（只返回 bool，不记录详情） ───────────────────────────────
bool colliding = scene->isStateColliding(*state, "arm");
// 第二参数指定规划组名，只检查该组内连杆

// ── 检查某点是否有效（包含碰撞 + 关节限位） ─────────────────────────────
bool valid = scene->isStateValid(*state, "", false);
// 第二参数：约束名称，空字符串 = 无约束
// 第三参数：verbose = false

// ── 允许碰撞矩阵（ACM） ───────────────────────────────────────────────────
collision_detection::AllowedCollisionMatrix & acm =
    scene->getAllowedCollisionMatrixNonConst();

// 查询
collision_detection::AllowedCollision::Type type;
bool found = acm.getAllowedCollision("link1", "link2", type);
// type: ALWAYS（始终允许）/ NEVER（始终禁止）/ CONDITIONAL（条件允许）

// 设置允许（如夹爪与抓取物体）
acm.setEntry("finger_left",  "target_box", true);
acm.setEntry("finger_right", "target_box", true);

// 恢复碰撞检测
acm.removeEntry("finger_left",  "target_box");
acm.removeEntry("finger_right", "target_box");

// 输出当前 ACM（调试）
acm.print(std::cout);
```

### 3.3 PlanningSceneInterface（简化接口）

```cpp
moveit::planning_interface::PlanningSceneInterface scene;

// ── 查询 ─────────────────────────────────────────────────────────────────
std::map<std::string, moveit_msgs::msg::CollisionObject> objects =
    scene.getObjects();
// 获取场景中所有碰撞物体，key = object.id

std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> attached =
    scene.getAttachedObjects();
// 获取已附着到机器人的物体

std::vector<std::string> known_objects = scene.getKnownObjectNames();
// 获取所有已知物体 ID

std::vector<std::string> objects_in_roi =
    scene.getKnownObjectNamesInROI(
        minx, miny, minz, maxx, maxy, maxz,
        true,  // with_type = false 则返回所有，true 返回指定类型
        {});   // 类型过滤列表（空=不过滤）

// ── 写入 ─────────────────────────────────────────────────────────────────
bool applyCollisionObject(
    const moveit_msgs::msg::CollisionObject & collision_object);
// ADD / REMOVE / APPEND / MOVE 操作由 collision_object.operation 决定
// operation = moveit_msgs::msg::CollisionObject::ADD      (0)
// operation = moveit_msgs::msg::CollisionObject::REMOVE   (1)
// operation = moveit_msgs::msg::CollisionObject::APPEND   (2)
// operation = moveit_msgs::msg::CollisionObject::MOVE     (3)

bool applyCollisionObjects(
    const std::vector<moveit_msgs::msg::CollisionObject> & collision_objects,
    const std::vector<moveit_msgs::msg::ObjectColor> & object_colors = {});
// 批量操作，object_colors 用于 RViz 可视化颜色

void removeCollisionObjects(const std::vector<std::string> & object_ids);

bool applyAttachedCollisionObject(
    const moveit_msgs::msg::AttachedCollisionObject & attached_object);

bool applyPlanningScene(const moveit_msgs::msg::PlanningScene & planning_scene);
// 最底层接口，直接发布完整场景差量
```

---

## 四、TF2 在 MoveIt2 中的用法

### 4.1 基础变换

```cpp
// 构造（ROS2 必须传 clock）
tf2_ros::Buffer tf_buffer(node->get_clock());
tf2_ros::TransformListener tf_listener(tf_buffer, node);

// lookupTransform：从 source_frame 到 target_frame 的变换
// 即：将 source_frame 中的点变换到 target_frame 中
geometry_msgs::msg::TransformStamped tf =
    tf_buffer.lookupTransform(
        "base_link",          // target_frame
        "link_tcp",           // source_frame
        tf2::TimePointZero,   // 最新时刻（等价于 ROS1 的 ros::Time(0)）
        tf2::durationFromSec(1.0)  // 超时（等价于 ros::Duration(1.0)）
    );
// tf.transform 包含 translation (x,y,z) 和 rotation (x,y,z,w)
// tf.header.frame_id = target_frame
// tf.child_frame_id = source_frame

// 同步等待 TF 可用（通常在初始化时）
bool can_transform = tf_buffer.canTransform(
    "base_link", "link_tcp", tf2::TimePointZero,
    tf2::durationFromSec(5.0)  // 最多等 5 秒
);

// ── 点变换 ───────────────────────────────────────────────────────────────
geometry_msgs::msg::PointStamped point_in;
point_in.header.frame_id = "link_tcp";
point_in.point.x = 0.0; point_in.point.y = 0.0; point_in.point.z = 0.1;

geometry_msgs::msg::PointStamped point_out;
tf2::doTransform(point_in, point_out, tf);

// ── 位姿变换 ─────────────────────────────────────────────────────────────
geometry_msgs::msg::PoseStamped pose_in, pose_out;
pose_in.header.frame_id = "link_tcp";
// 设置 pose_in.pose ...
tf2::doTransform(pose_in, pose_out, tf);
// pose_out.header.frame_id 自动设为 target_frame

// ── 四元数变换 ────────────────────────────────────────────────────────────
geometry_msgs::msg::QuaternionStamped q_in, q_out;
q_in.header.frame_id = "link_tcp";
tf2::doTransform(q_in, q_out, tf);
```

### 4.2 四元数操作

```cpp
// ROS 消息 ↔ tf2 四元数互转
tf2::Quaternion q_tf;
tf2::fromMsg(pose.orientation, q_tf);       // msg → tf2
pose.orientation = tf2::toMsg(q_tf);        // tf2 → msg

// 从 RPY 构造四元数
tf2::Quaternion q;
q.setRPY(roll, pitch, yaw);  // 内旋顺序 X-Y-Z（roll-pitch-yaw）
q.normalize();                // 归一化，防止浮点误差

// 四元数 → RPY
tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

// 四元数乘法（旋转叠加）
// dm_arm eefRotate：q_new = q_current * q_delta
tf2::Quaternion q_delta;
q_delta.setRPY(0, 0, angle_rad);  // 绕 Z 轴旋转
tf2::Quaternion q_new = q_current * q_delta;
q_new.normalize();

// 旋转矩阵 → 四元数
tf2::Matrix3x3 rot_matrix(
    xx, xy, xz,
    yx, yy, yz,
    zx, zy, zz
);
tf2::Quaternion q_from_mat;
rot_matrix.getRotation(q_from_mat);

// 四元数 → 旋转矩阵
tf2::Matrix3x3 mat(q_tf);
double r, p, y;
mat.getRPY(r, p, y);

// 注意：dm_arm 的 searchReachablePose 中构造末端 Z 轴方向的四元数
// 关键点：用从基座到目标点的单位向量作为 Z 轴，叉积构造正交基
tf2::Vector3 z_axis(px, py, pz);
z_axis.normalize();
tf2::Vector3 x_axis(1, 0, 0);
if (std::abs(z_axis.dot(x_axis)) > 0.9999) x_axis = tf2::Vector3(0, 1, 0);
tf2::Vector3 y_axis = z_axis.cross(x_axis).normalize();
// 注意顺序：y = z × x（右手系），然后 x = y × z
// dm_arm 直接用 z × x 得到 y，列向量排列构成旋转矩阵
```

---

## 五、ros2_control 关键 API

### 5.1 HardwareInfo 解析

```cpp
// on_init 参数 HardwareInfo 的主要字段：
hardware_interface::HardwareInfo info;

// 硬件级参数（来自 <hardware><param>）
info.hardware_parameters;       // std::unordered_map<std::string, std::string>

// 各关节信息
for (const hardware_interface::ComponentInfo & joint : info.joints) {
    joint.name;                  // 关节名称（与 URDF joint 名一致）
    joint.type;                  // "joint"
    joint.parameters;            // std::unordered_map<std::string, std::string>

    // 命令接口配置
    for (const hardware_interface::InterfaceInfo & ci : joint.command_interfaces) {
        ci.name;                 // "position" / "velocity" / "effort"
        ci.min;                  // 下限字符串（需 std::stod 转换）
        ci.max;                  // 上限字符串
        ci.initial_value;        // 初始值字符串
    }

    // 状态接口配置
    for (const hardware_interface::InterfaceInfo & si : joint.state_interfaces) {
        si.name;                 // "position" / "velocity" / "effort"
        si.initial_value;        // 状态初始值字符串
    }
}

// 传感器信息（用于仅发布状态的传感器）
info.sensors;                    // std::vector<ComponentInfo>

// 硬件名称（<ros2_control name="...">）
info.name;                       // "dm_arm_hardware"

// 硬件类型
info.type;                       // "system" / "sensor" / "actuator"
```

### 5.2 StateInterface 与 CommandInterface

```cpp
// StateInterface：只读，暴露给 controller_manager 和控制器
// 内部是指向 double 的指针
hardware_interface::StateInterface state_iface(
    joint_name,          // 关节名
    interface_type,      // "position"/"velocity"/"effort"
    &hw_position         // double* 指向实际数据
);

// CommandInterface：可读写
hardware_interface::CommandInterface cmd_iface(
    joint_name,
    "position",
    &hw_command          // double* 控制器写入目标，write() 读取
);

// 控制器通过 get_interface() 获取接口后直接操作指针指向的内存
// 所以 hw_positions_[i] 和 hw_commands_[i] 的内存地址在整个节点生命周期内不能改变
// 因此必须在 on_init 中 resize，不能 push_back（可能导致重新分配）
```

### 5.3 生命周期回调顺序

```
controller_manager 启动
    ↓
on_init()           ← 解析参数，检查配置，分配内存
    ↓
on_configure()      ← 打开串口，创建电机对象（不使能）
    ↓ (ros2 control list_controllers 此时显示 inactive)
on_activate()       ← 使能电机，读取初始状态
    ↓ (此时控制器可用，控制循环开始)
[read() → controller → write()] × N  ← 主循环（500Hz）
    ↓ (收到 deactivate 请求)
on_deactivate()     ← 平滑归零，失能电机
    ↓
on_cleanup()        ← 释放资源
    ↓
on_error()          ← 任意状态出错时调用（返回 ERROR 后节点仍可重新 configure）
```

**注意事项：**
- `on_init` 到 `on_activate` 之间不能做耗时操作（阻塞 controller_manager 启动）
- `read()` 和 `write()` 在实时线程调用，不能使用 `RCLCPP_INFO` 等非实时安全的日志（应用 `RCLCPP_INFO_THROTTLE` 或关闭日志）
- `hw_positions_` 向量大小在 `on_init` 后**不能改变**，否则 StateInterface 指针失效

---

## 六、服务调用注意事项（ROS2）

### 6.1 阻塞调用与 spin 的冲突

```cpp
// 危险：在同一 executor 的回调中调用 spin_until_future_complete 会死锁
// 因为回调占用了 executor 线程，spin 无法推进 future 完成

// 正确方式一：使用独立线程
void some_callback() {
    auto request = std::make_shared<SrvType::Request>();
    auto future = client->async_send_request(request);

    // 在独立线程中等待
    std::thread([future, this]() {
        if (rclcpp::spin_until_future_complete(node_, future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto res = future.get();
            // 处理结果
        }
    }).detach();
}

// 正确方式二：使用回调版 async_send_request
client->async_send_request(request,
    [this](rclcpp::Client<SrvType>::SharedFuture future) {
        auto res = future.get();
        // 处理结果（此回调在 executor 线程中运行）
    });

// 正确方式三：CallbackGroup（推荐用于 dm_arm_client）
// 为客户端和服务端分别创建 CallbackGroup，避免互相阻塞
auto client_callback_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
auto client = node->create_client<SrvType>("service",
    rmw_qos_profile_services_default, client_callback_group);
// 在 MultiThreadedExecutor 下，不同 CallbackGroup 可并发执行
```

### 6.2 服务端回调中调用 MoveIt

```cpp
// dm_arm_server 的 eefCmdCallback 中直接调用 eef_controller_->setGoalPoseBase()
// setGoalPoseBase 内部调用 arm_.plan() 和 arm_.execute()
// 这两个调用会阻塞（plan 最长 planning_time 秒，execute 最长 execution_timeout 秒）

// 在 MultiThreadedExecutor 下，长时间阻塞的回调不会阻塞其他服务的响应
// 但在 SingleThreadedExecutor 下，整个节点在 plan/execute 期间无响应

// 因此 dm_arm_server 必须使用 MultiThreadedExecutor：
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(server_node);
executor.spin();

// dm_arm_server 还需要为两个服务创建独立的 CallbackGroup
// 以允许 eef_cmd 和 task_planner 服务并发（虽然实际上只有一个规划组，顺序执行更安全）
auto eef_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
srv_eef_cmd_ = node->create_service<DmArmCmd>(
    "/dm_arm_server/eef_cmd",
    std::bind(&DmArmServer::eefCmdCallback, this, _1, _2),
    rmw_qos_profile_services_default,
    eef_group);
```

---

## 七、常见报错与解决

| 报错信息 | 原因 | 解决 |
|---------|------|------|
| `No MoveGroup configuration found` | move_group 节点未启动 | 检查 launch 文件中 move_group 节点，确认 SRDF/URDF 加载正常 |
| `Unable to load robot model` | `robot_description` 参数不存在或解析失败 | 确认 robot_state_publisher 已启动且 URDF 无语法错误（注意所有数值用浮点） |
| `Failed to get current robot state` | joint_states 未发布 | 确认 joint_state_broadcaster 已激活，`ros2 topic hz /joint_states` 检查频率 |
| `No IK solution` | 目标超出工作空间或奇异点 | 启用 allow_tweak + allow_feedforward，检查目标坐标距离（dm_arm 最大约 0.6m） |
| `Planning failed` | 规划器超时或无路径 | 增加 planning_time，增加 num_planning_attempts，检查碰撞场景，更换规划器 |
| `Controller action failed` | 控制器未激活或超时 | `ros2 control list_controllers` 确认状态，检查 joint_limits 设置 |
| `parameter 'xxx' has invalid type: expected [double] got [integer]` | URDF 或 joint_limits.yaml 中整数应写为浮点 | 将所有数值改为浮点，如 `100` → `100.0` |
| `QT error when loading URDF` | rviz_common 版本过高 | `sudo apt install ros-humble-rviz-common=<lower-version>` |
| `Could not find resource 'package://...'` | 包未安装或未 build | `colcon build --symlink-install`，确认 `install/` 中有 meshes |
| `setFromIK returned false` | IK 解算器超时或目标不可达 | 增加 `kinematics_solver_timeout`，尝试更换为 TracIK |
| `tf2::LookupException` | TF 树不完整或 TF 监听器未启动 | 确认 robot_state_publisher 发布了所有 TF，`ros2 run tf2_tools view_frames` 查看 |
| `move_group/DynamicReconfigure` service not available | ROS1 遗留代码调用了 ROS1 接口 | 检查代码中是否混用了 ROS1 API |
| 插件加载失败 `Could not load class` | pluginlib XML 路径或类名错误 | 检查 `dm_arm_hardware.xml` 中 `type` 属性与 `PLUGINLIB_EXPORT_CLASS` 的一致性 |

---

## 八、参数配置文件（dm_arm ROS2 版）

`config/dm_arm_config.yaml`（供 dm_arm_server 节点使用）：

```yaml
/**:
  ros__parameters:
    # ── MoveIt ───────────────────────────────────────────────────────
    moveit:
      planning_group: "arm"
      planner_id: "RRTConnect"
      planning_time: 5.0
      num_planning_attempts: 10
      allow_replanning: true
      execution_timeout: 15.0

    # ── 末端执行器 ────────────────────────────────────────────────────
    end_effector:
      max_reach: 0.6
      min_reach: 0.1
      velocity_scaling: 0.3
      acceleration_scaling: 0.3
      goal_position_tolerance: 0.015
      goal_orientation_tolerance: 0.05
      search:
        step: 5.0
        radius: 45.0
        max_iterations: 0      # 0 = 自动计算上限

    # ── 任务规划器 ────────────────────────────────────────────────────
    task_planner:
      enable_optimization: true
      pick_height_offset: 0.1
      place_height_offset: 0.1
      approach_distance: 0.05
      retreat_distance: 0.05
      default_wait_time: 0.5
```

**ROS2 参数文件关键注意事项：**
- 必须在 `/**:` 或 `/<node_name>:` 下添加 `ros__parameters:` 层（两个下划线）
- 参数名中的 `.` 分隔符在 YAML 中写为嵌套字典，在代码中写为 `"moveit.planning_group"`
- 整数和浮点类型严格区分，不能混用

---

## 九、moveit_controllers.yaml 关键配置

此文件决定 MoveIt 如何将规划好的轨迹发送给 ros2_control 控制器：

```yaml
# MoveIt 侧控制器配置（对接 ros2_control 侧的同名控制器）
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

arm_controller:
  type: FollowJointTrajectory
  # 对应 ros2_control 侧 arm_controller 的动作命名空间
  action_ns: follow_joint_trajectory
  default: true
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

gripper_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  default: true
  joints:
    - gripper_left

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
```

**工作原理：**
1. `arm_.execute(plan)` 调用 MoveIt 的 `TrajectoryExecutionManager`
2. `TrajectoryExecutionManager` 按关节名查找对应控制器（arm_controller 管理 joint1-6）
3. 通过 `FollowJointTrajectory` 动作向 `/arm_controller/follow_joint_trajectory` 发送目标
4. `arm_controller`（JointTrajectoryController）接收后执行，调用 `write()`，最终下达到电机

**关键限制：**
- MoveIt 侧的关节列表必须是 ros2_control 侧控制器关节列表的子集
- `action_ns` 必须与控制器的实际动作命名空间匹配（通常是 `follow_joint_trajectory`）
- gripper_right 是 mimic joint，不需要出现在任何控制器列表中
