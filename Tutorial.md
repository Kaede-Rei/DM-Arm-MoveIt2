[toc]

# 1. 安装

```bash
# 安装核心包和示例 Panda 机械臂
sudo apt install ros-humble-moveit \
				 ros-humble-moveit-setup-assistant \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-trajectory-controller \
                 ros-humble-moveit-simple-controller-manager \
                 ros-humble-moveit-resources-panda-moveit-config \
                 ros-humble-moveit-resources-panda-description
                 
# 更换为 MoveIt2 官方推荐的 DDS
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# 验证是否安装成功
ros2 pkg list | grep moveit
```

-   **教程 + 文档**：[MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)

# 2. Rviz

## 2.1. 启动演示及配置插件

-   终端输入 `ros2 launch moveit_resources_panda_moveit_config demo.launch.py` 启动机械臂演示：
-   <img src="Tutorial.assets/image-20260216110255042.png" alt="image-20260216110255042" style="zoom:50%;" />
-   `Add` 添加显示类型，一般基础的默认已存在，需要自行添加应用于 `MoveIt` 的主要是 `MotionPlanning`、`TF`、`Axes`

## 2.2. 显示类型

-   **MotionPlanning** 是 MoveIt 的专用 RViz 插件（Display Type），提供最完整的交互式运动规划界面。 主要功能包括：
    -   可视化并编辑 **Planning Scene**（规划场景，包括机器人当前状态、附加物体、碰撞物体、已知物体等）。
    -   交互设置 **Start State**（起始位姿，通常默认为当前机器人状态）和 **Goal State**（目标位姿，可通过拖拽末端执行器或设置关节角度交互修改）。
    -   选择规划组（Planning Group）、规划器（Planner）、规划参数，并一键 **Plan**（生成轨迹）。
    -   显示生成的 **Planned Path**（规划路径，通常以橙色/绿色线条或动画显示）。
    -   支持 **Execute**（执行规划路径到真实机器人或仿真）。
    -   包含多个子选项卡：Context（上下文配置）、Planning（规划设置）、Scene Objects（场景物体管理）、Query（查询起始/目标状态）、Status（规划状态反馈）。
    -   典型配置：Robot Description → "robot_description"；Trajectory Topic → "/move_group/display_planned_path" 或 "/display_planned_path"；Planning Scene Topic → "monitored_planning_scene" 或 "planning_scene"。 → 这是 MoveIt 用户最核心的插件，几乎所有交互式规划调试都依赖它。

-   **TF** 显示整个 TF（变换树）坐标系关系。 主要功能：

    -   以坐标轴形式可视化所有广播的坐标系（frame）及其父子关系（树状结构）。

    -   帮助快速检查坐标变换是否正确，例如 base_link → tool0 → wrist → end_effector 等链路。

    -   在 MoveIt 中非常重要：用于验证末端位姿、规划帧是否对齐、是否存在 TF 断链或延迟问题。 → 推荐始终添加，并启用 "Show Names" 和 "Show Axes" 以便观察。

-   **Axes** 显示一组固定的三维坐标轴（X红、Y绿、Z蓝）。 主要功能：

    -   作为参考坐标系，帮助判断当前 Fixed Frame 的方向和原点位置。

    -   在 MoveIt 调试中常用于确认规划帧（通常设为 /base_link 或 /world）的朝向是否正确。 → 简单但实用，常与 TF 一起使用作为全局参考。

-   **PlanningScene**（或 Scene Robot / Planning Scene Robot） 这是 **MotionPlanning** 插件内部的子显示选项（在 MotionPlanning 的 Displays 属性中展开）。 主要功能：

    -   显示当前规划场景中的机器人模型（Scene Robot）。
    -   可单独开关：Show Robot Visual（可视几何体）、Show Robot Collision（碰撞体，通常半透明）、Octree / Attached / World Objects（附加物体、世界物体）。 → 用于检查场景是否正确更新、碰撞检测是否生效。

-   **Trajectory**（或 Planned Path） 也是 **MotionPlanning** 插件内部的子显示选项（Planned Path 部分）。 主要功能：

    -   显示规划出的轨迹（通常为一系列关节插值点，以线条、动画或轨迹尾迹形式渲染）。

    -   支持 Loop Animation（循环播放轨迹）、Fade Animation（渐隐效果）、轨迹颜色/粗细调整。

    -   可通过 Trajectory Slider（Panel → Motion Planning → Trajectory Slider）手动拖动预览轨迹的每一步。 → 规划成功后，这是最直观的轨迹可视化结果。

# 3. URDF

## 3.1. 模型与 urdf 文件

-   **目录结构：**

    ```bash
    model/
    ├── meshes/
    |	├── collision/
    |	|	├── base_link.stl
    |	|	└── xxx.stl
    |	└── visual/
    |		├── base_link.dae
    |		└── xxx.dae
    └──	urdf/
    	└── <robot-name>_description.urdf.xacro		(如原来就有 urdf 则继续使用 .urdf)
    ```

>   *注意 urdf 里所有参数都要用浮点数，原 MoveIt1 可用而 MoveIt2 不可用*

## 3.2. 机器人描述包 + Rviz 验证

-   **创建描述包：** 

    ```bash
    # 如果未安装相关依赖，则先安装
    sudo apt install ros-humble-joint-state-publisher-gui
    # 进入 src
    cd src
    # 创建
    ros2 pkg create --build-type ament_cmake <robot-name>_description --dependencies urdf xacro robot_state_publisher joint_state_publisher_gui
    ```

-   **目录结构：**

    ```bash
    <robot-name>_description/
    ├── CMakeLists.txt
    ├── package.xml
    ├── meshes/
    ├── urdf/
    │   ├── <robot-name>_description.urdf.xacro		← 主文件（xacro 或 urdf）
    │   ├── materials.xacro							← 可选
    │   ├── macros/									← 可选
    │   └── ...
    ├── launch/
    │   └── display.launch.py						← 用于 rviz 可视化检查
    └── config/
        └── joint_names.yaml						← 可选
    ```

-   **修改 CmakeLists.txt：**

    ```cmake
    cmake_minimum_required(VERSION 3.8)
    project(dm_arm_description)
    
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
    
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(urdf REQUIRED)
    find_package(xacro REQUIRED)
    find_package(robot_state_publisher REQUIRED)
    
    ##########################################################
    ## 注释掉 joint_state_publisher_gui，因为是运行时节点而非依赖 ##
    ### find_package(joint_state_publisher_gui REQUIRED)   ###
    ##########################################################
    
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # the following line skips the linter which checks for copyrights
      # comment the line when a copyright and license is added to all source files
      set(ament_cmake_copyright_FOUND TRUE)
      # the following line skips cpplint (only works in a git repo)
      # comment the line when this package is in a git repo and when
      # a copyright and license is added to all source files
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
    endif()
    
    ############################
    ## 安装 URDF、meshes 等资源 ##
    ############################
    install(
      DIRECTORY urdf meshes launch config
      DESTINATION share/${PROJECT_NAME}
    )
    
    ament_package()
    
    ```

-   **添加 display.launch.py：**

    ```python
    from launch import LaunchDescription
    from launch.substitutions import Command, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    
    URDF_PATH = "dm_arm_description.urdf"
    
    
    def generate_launch_description():
        # 查找包路径
        pkg_share = FindPackageShare("dm_arm_description").find("dm_arm_description")
    
        # URDF 文件路径
        urdf_path = PathJoinSubstitution([pkg_share, "urdf", URDF_PATH])
    
        # 判断是否使用 xacro
        use_xacro = True if URDF_PATH.endswith(".xacro") else False
    
        return LaunchDescription(
            [
                # 从 URDF 加载并发布 robot_description
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[
                        {
                            "robot_description": (
                                Command(["xacro ", urdf_path])
                                if use_xacro
                                else Command(["cat ", urdf_path])
                            )
                        }
                    ],
                ),
                # 启动 joint_state_publisher_gui
                Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    name="joint_state_publisher_gui",
                    output="screen",
                ),
                # 启动 RViz 并加载配置
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=[
                        "-d",
                        PathJoinSubstitution([pkg_share, "config", "view_robot.rviz"]),
                    ],
                ),
            ]
        )
    
    ```

-   **构建并 source：** `colcon build --symlink-install --packages-select <robot-name>_description`

-   **运行 display.launch.py：** `ros2 launch dm_arm_description display.launch.py `

    -   选择 `Global Options/Fixed Frame` 为基座：<img src="Tutorial.assets/image-20260217120130722.png" alt="image-20260217120130722" style="zoom: 50%;" />

    -   添加 `RobotModel` 并选择 `RobotModel/Description Topic` 为 `/robot_description`

        >   *如果没显示则是未及时更新，可点一下其他选项刷新*

## 3.3. MoveIt Config 包

### 3.3.1. MoveIt Setup Assistant

-   **启动：** `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
-   **新建：** `Create New` -> `Browse` -> `<robot-name>_description.urdf` -> `Load File`
-   **Self-Collisions：** `Generate Collision Matirx`
-   **Planning Groups：** `Add Group`
    -   **arm：** `KDL` -> `RRTConnect` -> `Add Kin. Chain` (`base_link` -> `link_tcp` 即底座 - 末端工作点)
    -   **gripper：** `Add Joints` -> `gripper_left` （`gripper_right` 是 mimic joint 不需要加进来）
-   **Robot Poses：** `Add` -> `home`
-   **End Effectors：** `Add` -> `Name=gripper` -> `Group=gripper` -> `Link=link_tcp` -> `Parent=arm`
-   **ROS2 Controllers：** `Auto` 
    -   **arm：** `joint_trajectory_controller/JointTrajectoryController`
    -   **gripper：** `position_controllers/GripperActionController`

-   **MoveIt Controllers：** `Auto`
    -   **arm：** `FollowJointTrajectory`
    -   **gripper：** `GripperCommand`

-   剩下个必填项为作者信息，按实际情况填即可
-   在 `src/` 下新建 `<robot-name>_moveit_config/` 包用于存放生成的 SRDF，保存后构建并 `Source`
-   `ros2 launch dm_arm_moveit_config demo.launch.py` 验证

>   -   *如果 setup_assistant 加载 urdf file 时报错 QT 相关的错误，需要降级 ros-humble-rviz-common*
>   -   *如果仍然有 parameter 'robot_description_planning.joint_limits.joint1.max_velocity'*
>       *has invalid type: expected [double] got [integer] 报错，则在 config/joint_limits.yaml*
>       *里将整数补齐为浮点数后再构建并 Source*

### 3.3.2. MoveIt Setup Assistant 其他可选选项



### 3.3.3. 生成文件结构说明

Setup Assistant 完成后会在 `<robot-name>_moveit_config/config/` 下生成以下关键文件，了解它们的作用有助于后续手动调整：

| 文件                      | 作用                                       |
| ------------------------- | ------------------------------------------ |
| `<robot>.srdf`            | 规划组定义、碰撞豁免、预设姿态、末端执行器 |
| `joint_limits.yaml`       | 关节速度/加速度上限，覆盖 URDF 中的值      |
| `kinematics.yaml`         | IK 求解器配置（插件名、超时、搜索步长）    |
| `ompl_planning.yaml`      | OMPL 规划器参数，默认使用 RRTConnect       |
| `ros2_controllers.yaml`   | ros2_control 的 controller 定义            |
| `moveit_controllers.yaml` | MoveIt 侧的 controller 接口配置            |
| `moveit.rviz`             | RViz 预配置，含 MotionPlanning 插件        |

### 3.3.4. kinematics.yaml 关键参数

生成后可手动微调 `config/kinematics.yaml`，KDL 的两个参数影响 IK 成功率：

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005   	# 搜索步长，越小越精确但越慢
  kinematics_solver_timeout: 0.005             	# 单次 IK 超时（秒），复杂位姿可调到 0.05
  kinematics_solver_attempts: 3               	# 超时后重试次数
```

如果 KDL 在某些奇异构型下 IK 频繁失败，可以将求解器换为 `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`（需额外安装 `ros-humble-trac-ik-kinematics-plugin`），参数格式相同，收敛性更好，对近奇异点的处理比 KDL 稳定

### 3.3.5. 仿真与真实臂的切换

Setup Assistant 生成的 `ros2_controllers.yaml` 默认使用 `mock_components/GenericSystem`（即 FakeSystem）用于仿真；切换到真实臂时**不需要重新跑 Setup Assistant**，只需替换 hardware interface：

```yaml
# 仿真（config/ros2_controllers.yaml 或 description 包内的 .ros2_control.xacro）
ros2_control:
  hardware:
    plugin: mock_components/GenericSystem   # FakeSystem，纯仿真

# 真实臂（后续重构硬件接口后替换为）
ros2_control:
  hardware:
    plugin: dm_arm_hardware/DmHardwareInterface
    # 其余 joint 定义、SRDF、Planning Group 完全不变
```

MoveIt 规划层（SRDF、kinematics.yaml、ompl_planning.yaml）与硬件接口完全解耦，切换硬件不影响规划配置

### 3.3.6. mimic joint 处理

`gripper_right` 在 URDF 中声明了 `<mimic joint="gripper_left">`，但 `mock_components/GenericSystem` 默认不处理 mimic 关系，仿真时两指可能不同步。在 `ros2_controllers.yaml` 的 gripper controller 下添加：

```yaml
gripper_controller:
  ros__parameters:
    joints:
      - gripper_left
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: true
```

## 3.4. 纯 urdf 迁移为 xacro

-   **目的：**把 `.urdf` 改为 `.urdf.xacro`，分离出 `dm_arm.ros2_control.xacro`，通过 launch 参数控制仿真/真实切换，方便后续维护和调试
-   **改造步骤：**把 URDF 主文件改成 xacro 格式 -> 新建一个专门放 ros2_control 描述的 xacro 文件 -> 重新对 xacro 文件再配置一次 MoveIt Config 

### 3.4.1.把 URDF 主文件改成 xacro 格式

-   **重命名：** `<robot-name>_description.urdf.xacro`

-   然后在文件**第二行**（`<robot>` 标签里）加上 xacro 命名空间声明：

    ```xml
    <!-- 原来 -->
    <robot name="<robot-name>_description">
    
    <!-- 改为 -->
    <robot name="<robot-name>_description" xmlns:xacro="http://www.ros.org/wiki/xacro">
    ```

-   然后在文件**末尾** `</robot>` 前加两行，引入 ros2_control 描述：

    ```bash
    <!-- 引入 ros2_control 描述 -->
      <xacro:arg name="use_fake_hardware" default="true"/>
      <xacro:include filename="$(find <robot-name>_description)/urdf/<robot-name>.ros2_control.xacro"/>
      <xacro:<robot-name>_ros2_control name="<robot-name>_hardware" use_fake_hardware="$(arg use_fake_hardware)"/>
    
    </robot>
    ```

### 3.4.2. 新建 \<robot-name\>.ros2_control.xacro

在 `urdf/` 下新建文件并添加：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:macro name="dm_arm_ros2_control" params="name use_fake_hardware:=true">

        <ros2_control name="${name}" type="system">
            <!-- 硬件插件选择 -->
            <hardware>
                <!-- 仿真模式 -->
                <xacro:if value="${use_fake_hardware}">
                    <plugin>mock_components/GenericSystem</plugin>
                </xacro:if>
                <!-- 真实硬件模式 -->
                <xacro:unless value="${use_fake_hardware}">
                    <plugin>dm_arm_hardware/DmHardwareInterface</plugin>
                    <param name="serial_port">/dev/ttyACM0</param>
                    <param name="baudrate">921600</param>
                    <param name="control_frequency">500.0</param>
                </xacro:unless>
            </hardware>

            <!-- 关节名称（必须与 URDF 里的 joint name 一致） -->
            <joint name="joint1">
                <command_interface name="position"/>
                <state_interface name="position">
                    <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
                <state_interface name="effort"/>
            </joint>
			<!-- joint2 ~ joint6 -->
            <joint name="gripper_left">
                <command_interface name="position"/>
                <state_interface name="position">
                    <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
                <state_interface name="effort"/>
            </joint>
        </ros2_control>

    </xacro:macro>

</robot>
```

### 3.4.3. 重复之前的操作配置 MoveIt Config



# 4. 基础空间规划

## 4.1. 设置目标关节角度

### 4.1.1. 依赖

```xml
    <depend>rclcpp</depend>
    <depend>moveit_ros_planning_interface</depend>
```

-   **VSCode 路径：**Ctrl + Shift + P 并输入 `C/C++` 选择编辑配置(JSON)：

    ```json
                "includePath": [
                    "${workspaceFolder}/**",
                    "/opt/ros/humble/include/**",
                    "/usr/include/**"
                ],
    ```

-   **include：** `#include <rclcpp/rclcpp.hpp>`  `#include <moveit/move_group_interface/move_group_interface.h>`

### 4.1.2. 步骤与 API

-   **步骤：** `新建节点` -> `单开线程并 Node Spin` -> `创建 MoveGroupInterface 对象` -> `设置目标关节+规划执行`

-   **API：**

    ```cpp
    # 节点
    auto node = rclcpp::Node::make_shared("end_effector_cmd");
    
    # 线程
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });
    
    # MoveGroupInterface 对象
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    
    # 获取关节角与关节名称
    std::vector<double> current_joints = arm.getCurrentJointValues();
    std::vector<std::string> joint_names = arm.getJointNames();
    
    # 设置目标关节角
    bool success = arm.setJointValueTarget(target_joints);
    
    # 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode err_code = arm.plan(plan);
    
    # 执行
    err_code = arm.execute(plan);
    
    # 关闭节点 + 终止线程
    rclcpp::shutdown();
    spin_thread.join();
    ```

## 4.2. 设置目标位姿

### 4.2.1. 传入 MoveIt2 所有配置参数：

-   在 MoveIt 2 (ROS 2) 中，**全局参数服务器（Parameter Server）已不存在**；每个节点都是独立的实体，必须在启动时显式获取其所需的全部上下文参数：

    ```python
    # run.launch.py
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from moveit_configs_utils import MoveItConfigsBuilder
    
    
    def generate_launch_description():
        # 构建 MoveIt 配置
        moveit_config = MoveItConfigsBuilder(
            "dm_arm_description", package_name="dm_arm_moveit_config"
        ).to_moveit_configs()
    
        # 创建节点并传入 moveit_config.to_dict()
        node = Node(
            package="dm_arm_controller",
            executable="end_effector_cmd",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True},
            ],
        )
    
        return LaunchDescription([node])
    
    ```

### 4.2.2. 设置目标 Pose / Position / Orientation

```cpp
using TargetVariant = std::variant<
    geometry_msgs::msg::Pose,
    geometry_msgs::msg::Point,
    geometry_msgs::msg::Quaternion
>;
/**
 * @brief 设置末端执行器的目标位姿、位置或姿态
 * @param target 目标位姿、位置或姿态(geometry_msgs::msg::Pose / geometry_msgs::msg::Point / geometry_msgs::msg::Quaternion)
 * @return 设置是否成功
 */
bool EndEffectorCmd::set_end(const TargetVariant& target) {
    bool success = false;

    if(auto* pose = std::get_if<geometry_msgs::msg::Pose>(&target)) {
        success = _arm_.setPoseTarget(*pose);
    }
    else if(auto* point = std::get_if<geometry_msgs::msg::Point>(&target)) {
        success = _arm_.setPositionTarget(point->x, point->y, point->z);
    }
    else if(auto* quat = std::get_if<geometry_msgs::msg::Quaternion>(&target)) {
        success = _arm_.setOrientationTarget(quat->x, quat->y, quat->z, quat->w);
    }
    else { return false; }

    return success;
}
```



