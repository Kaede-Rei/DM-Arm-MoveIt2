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
-   <img src="MoveIt-Tutorials.assets/image-20260216110255042.png" alt="image-20260216110255042" style="zoom:50%;" />
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

## 3.2. 机器人描述包并在 Rviz 里验证

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
    ```

-   **构建并 source：** `colcon build --symlink-install --packages-select <robot-name>_description`

-   **运行 display.launch.py：** `ros2 launch dm_arm_description display.launch.py `

    -   选择 `Global Options/Fixed Frame` 为基座：<img src="Tutorial.assets/image-20260217120130722.png" alt="image-20260217120130722" style="zoom: 50%;" />

    -   添加 `RobotModel` 并选择 `RobotModel/Description Topic` 为 `/robot_description`

        >   *如果没显示则是未及时更新，可点一下其他选项刷新*

