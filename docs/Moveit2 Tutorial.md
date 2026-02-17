# dm_arm ROS2 Humble + MoveIt2 完整开发教程

> 基于 dm_arm 项目从 ROS1 Noetic 迁移至 ROS2 Humble，涵盖硬件接口 → 运动规划 → 障碍物规避 → 单点搜索 → 多点任务规划 → 服务封装的完整流程。
>
> **前提：已完成 ① dm_arm_description 包（URDF + RViz2 验证）② MoveIt Setup Assistant 配置。**

---

## 阶段三：ros2_control 硬件接口

### 3.1 ros2_control 架构

ROS1 的 `ros_control` 在 ROS2 中被 `ros2_control` 替代，架构发生了根本变化。

```
ROS1 ros_control                    ROS2 ros2_control
─────────────────────               ─────────────────────────────────────
hardware_interface::RobotHW    →    hardware_interface::SystemInterface
                                    hardware_interface::SensorInterface
                                    hardware_interface::ActuatorInterface

controller_manager::ControllerManager  →  controller_manager::ControllerManager (重写)

JointStateInterface                →  StateInterface (position/velocity/effort)
PositionJointInterface             →  CommandInterface (position/velocity/effort)

ros_control 直接构造                →  通过 pluginlib 动态加载插件
```

**ROS1 的 `RobotHW` 类接口：**
```cpp
// 旧版（dm_hardware_interface.h）
class DMHardwareInterface : public hardware_interface::RobotHW {
public:
    bool init();
    void read();
    void write();
};
```

**ROS2 的 `SystemInterface` 类接口（必须实现的纯虚函数）：**
```cpp
// 新版
class DmHardwareInterface : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
};
```

**关键差异：**
- `init()` → `on_init()`，参数从 NodeHandle 变为 `HardwareInfo`（从 URDF ros2_control 标签解析）
- `read()/write()` 增加 `time` 和 `period` 参数
- 接口注册从 `registerInterface()` 变为返回 `StateInterface`/`CommandInterface` 向量
- 硬件参数从 yaml 参数服务器移至 **URDF 的 `<ros2_control>` 标签**

---

### 3.2 URDF 集成 ros2_control 标签

在 `dm_arm_description` 包中，需要将 ros2_control 的硬件描述嵌入 URDF 或单独的 xacro 文件中。

**方式一：独立 xacro 文件（推荐）**

创建 `urdf/dm_arm.ros2_control.xacro`：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="dm_arm_ros2_control" params="name use_fake_hardware:=^|false">

    <ros2_control name="${name}" type="system">

      <hardware>
        <xacro:if value="${use_fake_hardware}">
          <!-- 仿真模式：MoveIt Setup Assistant 默认生成此配置 -->
          <plugin>mock_components/GenericSystem</plugin>
          <!-- mimic joint 仿真支持 -->
          <param name="mock_sensor_commands">false</param>
        </xacro:if>
        <xacro:unless value="${use_fake_hardware}">
          <!-- 真实硬件：替换为自定义插件 -->
          <plugin>dm_arm_hardware/DmHardwareInterface</plugin>
          <!-- 硬件参数直接嵌入 URDF，而非 yaml -->
          <param name="serial_port">/dev/ttyACM0</param>
          <param name="baudrate">921600</param>
          <param name="control_frequency">500.0</param>
          <param name="use_mit_mode">false</param>
          <param name="kp">30.0</param>
          <param name="kd">1.0</param>
          <param name="max_position_change">0.5</param>
          <param name="max_velocity">3.0</param>
        </xacro:unless>
      </hardware>

      <!-- 6 个旋转关节，与 URDF joint 名称完全对应 -->
      <joint name="joint1">
        <command_interface name="position">
          <param name="min">-2.0943951023931953</param>
          <param name="max">2.0943951023931953</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <!-- 真实硬件专属参数（on_init 时通过 HardwareInfo 读取） -->
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">1</param>
          <param name="motor_type">2</param>
        </xacro:unless>
      </joint>

      <joint name="joint2">
        <command_interface name="position">
          <param name="min">0.0</param>
          <param name="max">3.141592653589793</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">2</param>
          <param name="motor_type">2</param>
        </xacro:unless>
      </joint>

      <joint name="joint3">
        <command_interface name="position">
          <param name="min">0.0</param>
          <param name="max">4.71238898038469</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">3</param>
          <param name="motor_type">2</param>
        </xacro:unless>
      </joint>

      <joint name="joint4">
        <command_interface name="position">
          <param name="min">-1.5707963267948966</param>
          <param name="max">1.5707963267948966</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">4</param>
          <param name="motor_type">0</param>
        </xacro:unless>
      </joint>

      <joint name="joint5">
        <command_interface name="position">
          <param name="min">-1.5707963267948966</param>
          <param name="max">1.5707963267948966</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">5</param>
          <param name="motor_type">0</param>
        </xacro:unless>
      </joint>

      <joint name="joint6">
        <command_interface name="position">
          <param name="min">-3.141592653589793</param>
          <param name="max">3.141592653589793</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">6</param>
          <param name="motor_type">0</param>
        </xacro:unless>
      </joint>

      <!-- gripper_left：直线导轨，prismatic joint，单位 m -->
      <joint name="gripper_left">
        <command_interface name="position">
          <param name="min">-0.045</param>
          <param name="max">0.0</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
        <xacro:unless value="${use_fake_hardware}">
          <param name="motor_id">7</param>
          <param name="motor_type">0</param>
          <!-- 导轨传动比：m / rad，用于单位换算 -->
          <param name="lead">0.053</param>
        </xacro:unless>
      </joint>

      <!-- gripper_right 是 mimic joint，mock_components 不处理 mimic -->
      <!-- 真实硬件时 gripper_right 跟随 gripper_left 在驱动层同步，无需单独控制 -->

    </ros2_control>
  </xacro:macro>
</robot>
```

**在主 xacro 中引用（需将 urdf 改为 xacro 格式）：**
```xml
<!-- dm_arm_description.urdf.xacro -->
<?xml version="1.0"?>
<robot name="dm_arm_description" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:arg name="use_fake_hardware" default="false"/>

  <!-- 包含 ros2_control 定义 -->
  <xacro:include filename="$(find dm_arm_description)/urdf/dm_arm.ros2_control.xacro"/>
  <xacro:dm_arm_ros2_control name="dm_arm_hardware" use_fake_hardware="$(arg use_fake_hardware)"/>

  <!-- ... 其余 link/joint 定义不变 ... -->
</robot>
```

---

### 3.3 硬件接口包结构

创建 `dm_arm_hardware` 包：

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake dm_arm_hardware \
    --dependencies rclcpp hardware_interface pluginlib
```

目录结构：
```
dm_arm_hardware/
├── CMakeLists.txt
├── package.xml
├── dm_arm_hardware.xml          ← pluginlib 插件描述
├── include/
│   └── dm_arm_hardware/
│       ├── dm_hardware_interface.hpp
│       └── visibility_control.hpp
└── src/
    ├── dm_hardware_interface.cpp
    └── SerialPort.cpp           ← 可直接复用 ROS1 版本
```

**package.xml：**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>dm_arm_hardware</name>
  <version>0.1.0</version>
  <description>dm_arm ros2_control hardware interface</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>

  <export>
    <build_type>ament_cmake</build_type>
    <!-- 声明插件 -->
    <hardware_interface plugin="${prefix}/dm_arm_hardware.xml"/>
  </export>
</package>
```

**dm_arm_hardware.xml（pluginlib 插件描述）：**
```xml
<library path="dm_arm_hardware">
  <class name="dm_arm_hardware/DmHardwareInterface"
         type="dm_arm_hardware::DmHardwareInterface"
         base_class_type="hardware_interface::SystemInterface">
    <description>dm_arm ROS2 hardware interface using Damiao motors</description>
  </class>
</library>
```

---

### 3.4 硬件接口头文件

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

// 可直接复用 ROS1 的底层驱动头文件（无 ROS 依赖部分）
#include "dm_arm_hardware/damiao.h"
#include "dm_arm_hardware/SerialPort.h"

namespace dm_arm_hardware
{

class DmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DmHardwareInterface)

  // ── 生命周期回调 ──────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── 接口导出 ──────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── 读写 ─────────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void return_zero_smooth();  // 等价于 ROS1 版的 returnZero()

  // 从 HardwareInfo 解析的配置
  std::string serial_port_;
  int baudrate_;
  double control_frequency_;
  bool use_mit_mode_;
  double kp_, kd_;
  double max_position_change_;
  double max_velocity_;

  // 每个关节的电机配置（从 URDF <joint><param> 读取）
  struct JointConfig {
    std::string name;
    int motor_id;
    int motor_type;
    double lead = 0.0;  // 仅 gripper_left 非零，单位 m/rad
  };
  std::vector<JointConfig> joint_configs_;

  // 关节状态/命令（与 StateInterface/CommandInterface 共享内存）
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

### 3.5 硬件接口实现

`src/dm_hardware_interface.cpp`：

```cpp
#include "dm_arm_hardware/dm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

// 注册插件（必须在 .cpp 文件末尾）
PLUGINLIB_EXPORT_CLASS(
  dm_arm_hardware::DmHardwareInterface,
  hardware_interface::SystemInterface)

namespace dm_arm_hardware
{

static constexpr double TWO_PI = 2.0 * M_PI;

// ─────────────────────────────────────────────────────────
//  on_init：解析 HardwareInfo，读取参数，不做硬件操作
//  等价于 ROS1 的 init() 中的"读取参数"部分
// ─────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用基类，检查 URDF 结构合法性
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── 读取硬件级参数（来自 URDF <hardware><param>） ──────────
  // info_.hardware_parameters 是 std::unordered_map<std::string, std::string>
  serial_port_   = info_.hardware_parameters.count("serial_port")
                   ? info_.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
  baudrate_      = info_.hardware_parameters.count("baudrate")
                   ? std::stoi(info_.hardware_parameters.at("baudrate")) : 921600;
  control_frequency_ = info_.hardware_parameters.count("control_frequency")
                       ? std::stod(info_.hardware_parameters.at("control_frequency")) : 500.0;
  use_mit_mode_  = info_.hardware_parameters.count("use_mit_mode")
                   ? (info_.hardware_parameters.at("use_mit_mode") == "true") : false;
  kp_            = info_.hardware_parameters.count("kp")
                   ? std::stod(info_.hardware_parameters.at("kp")) : 30.0;
  kd_            = info_.hardware_parameters.count("kd")
                   ? std::stod(info_.hardware_parameters.at("kd")) : 1.0;
  max_position_change_ = info_.hardware_parameters.count("max_position_change")
                         ? std::stod(info_.hardware_parameters.at("max_position_change")) : 0.5;
  max_velocity_  = info_.hardware_parameters.count("max_velocity")
                   ? std::stod(info_.hardware_parameters.at("max_velocity")) : 3.0;

  // ── 解析每个关节配置（来自 URDF <joint><param>） ─────────
  // info_.joints 对应 URDF <ros2_control> 下的所有 <joint>
  for (const auto & joint : info_.joints) {
    JointConfig cfg;
    cfg.name = joint.name;

    // joint.parameters 是 std::unordered_map<std::string, std::string>
    cfg.motor_id   = joint.parameters.count("motor_id")
                     ? std::stoi(joint.parameters.at("motor_id")) : 0;
    cfg.motor_type = joint.parameters.count("motor_type")
                     ? std::stoi(joint.parameters.at("motor_type")) : 0;
    cfg.lead       = joint.parameters.count("lead")
                     ? std::stod(joint.parameters.at("lead")) : 0.0;

    joint_configs_.push_back(cfg);

    // 检查 command/state interface 配置
    // info_.joints[i].command_interfaces / state_interfaces
    // 每个 interface 有 .name ("position"/"velocity") 和 .parameters
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
        "Joint '%s' must have exactly one 'position' command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 分配状态/命令缓冲区
  size_t n = joint_configs_.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);
  hw_commands_prev_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"),
    "on_init OK: %zu joints, port=%s, baud=%d", n, serial_port_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────
//  export_state_interfaces：将状态缓冲区暴露给 controller_manager
//  ROS1 等价：joint_state_interface_.registerHandle()
// ─────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
DmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const std::string & name = joint_configs_[i].name;

    // StateInterface(joint_name, interface_type, pointer_to_double)
    // interface_type 是字符串常量 "position" / "velocity" / "effort"
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(name, hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]));
  }

  return state_interfaces;
}

// ─────────────────────────────────────────────────────────
//  export_command_interfaces：将命令缓冲区暴露给 controller_manager
//  ROS1 等价：position_joint_interface_.registerHandle()
// ─────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
DmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const std::string & name = joint_configs_[i].name;

    // CommandInterface(joint_name, interface_type, pointer_to_double)
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

// ─────────────────────────────────────────────────────────
//  on_configure：打开串口，创建电机对象
//  ROS1 等价：init() 中"创建串口和电机"部分
// ─────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    serial_ = std::make_shared<SerialPort>(serial_port_, baudrate_);
    motor_controller_ = std::make_shared<damiao::Motor_Control>(serial_);
    RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"), "Serial %s opened", serial_port_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Serial open failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 创建电机对象（不使能，仅注册）
  for (const auto & cfg : joint_configs_) {
    auto motor = std::make_shared<damiao::Motor>(
      static_cast<damiao::DM_Motor_Type>(cfg.motor_type), cfg.motor_id, 0x00);
    motors_.push_back(motor);
    motor_controller_->addMotor(motor.get());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────
//  on_activate：使能电机，读取初始位置，切换控制模式
//  ROS1 等价：init() 中"使能电机"和"读取初始位置"部分
// ─────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 使能电机
  for (size_t i = 0; i < motors_.size(); ++i) {
    try {
      motor_controller_->enable(*motors_[i]);
      RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"),
        "Motor %s (ID=%d) enabled", joint_configs_[i].name.c_str(), joint_configs_[i].motor_id);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
        "Enable motor %s failed: %s", joint_configs_[i].name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // joint1 使用 MIT 模式，其余切换为位置速度模式
  if (!use_mit_mode_) {
    for (size_t i = 1; i < motors_.size(); ++i) {
      motor_controller_->switchControlMode(*motors_[i], damiao::POS_VEL_MODE);
      usleep(200 * 1000);
    }
  }

  // 读取初始位置（多次平均）
  usleep(200 * 1000);
  constexpr int READ_COUNT = 5;
  std::vector<double> pos_sum(joint_configs_.size(), 0.0);

  for (int j = 0; j < READ_COUNT; ++j) {
    read(rclcpp::Time{}, rclcpp::Duration{0, 0});
    for (size_t i = 0; i < joint_configs_.size(); ++i) pos_sum[i] += hw_positions_[i];
    usleep(20 * 1000);
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    hw_positions_[i] = pos_sum[i] / READ_COUNT;
    hw_commands_[i] = hw_positions_[i];
    hw_commands_prev_[i] = hw_positions_[i];
    RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"),
      "Joint %s init pos: %.3f rad", joint_configs_[i].name.c_str(), hw_positions_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────
//  on_deactivate：平滑归零，失能电机
//  ROS1 等价：析构函数中的 returnZero() + disable()
// ─────────────────────────────────────────────────────────
hardware_interface::CallbackReturn DmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return_zero_smooth();

  for (size_t i = 0; i < motors_.size(); ++i) {
    try {
      motor_controller_->disable(*motors_[i]);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"),
        "Disable motor %s failed: %s", joint_configs_[i].name.c_str(), e.what());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motors_.clear();
  motor_controller_.reset();
  serial_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("DmHardwareInterface"), "Hardware error occurred");
  return hardware_interface::CallbackReturn::ERROR;
}

// ─────────────────────────────────────────────────────────
//  read：从电机读取状态
//  ROS1 等价：DMHardwareInterface::read()
//  period.seconds() 是上次 read 到本次的时间间隔，可用于速度估计
// ─────────────────────────────────────────────────────────
hardware_interface::return_type DmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motors_.size(); ++i) {
    try {
      double pos = motors_[i]->Get_Position();

      // gripper_left 是直线导轨，电机弧度 → 线位移（m）
      if (joint_configs_[i].lead > 0.0) {
        pos = pos / TWO_PI * joint_configs_[i].lead;
      }

      // 异常值保护
      if (std::isnan(pos) || std::isinf(pos)) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DmHardwareInterface"),
          *rclcpp::Clock::make_shared(), 1000,
          "Joint %s: invalid position, using previous", joint_configs_[i].name.c_str());
        pos = hw_commands_prev_[i];
      }

      hw_positions_[i]  = pos;
      hw_velocities_[i] = motors_[i]->Get_Velocity();
      hw_efforts_[i]    = motors_[i]->Get_tau();

    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DmHardwareInterface"),
        *rclcpp::Clock::make_shared(), 1000,
        "Read motor %s error: %s", joint_configs_[i].name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────
//  write：向电机发送命令
//  ROS1 等价：DMHardwareInterface::write()
//  period.seconds() 替代了 1.0 / control_frequency_
// ─────────────────────────────────────────────────────────
hardware_interface::return_type DmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) dt = 1.0 / control_frequency_;

  for (size_t i = 0; i < motors_.size(); ++i) {
    try {
      const bool is_gripper = (joint_configs_[i].lead > 0.0);
      const double scale_to_motor = is_gripper ? (TWO_PI / joint_configs_[i].lead) : 1.0;

      double cmd_motor  = hw_commands_[i]      * scale_to_motor;
      double prev_motor = hw_commands_prev_[i]  * scale_to_motor;
      double meas_motor = hw_positions_[i]      * scale_to_motor;

      double position_change = cmd_motor - prev_motor;

      // 安全限制：单步最大变化
      if (std::abs(position_change) > max_position_change_) {
        position_change = std::copysign(max_position_change_, position_change);
        cmd_motor = prev_motor + position_change;
      }

      // 计算目标速度
      double target_vel;
      if (use_mit_mode_) {
        target_vel = position_change / dt;
      } else {
        // 位置速度模式：跟踪误差 + 前馈速度
        double pos_err = cmd_motor - meas_motor;
        target_vel = 10.0 * pos_err;
        if (std::abs(position_change) > 1e-4) target_vel += position_change / dt;
      }

      // 限速
      target_vel = std::clamp(target_vel, -max_velocity_, max_velocity_);

      // 下发
      if (use_mit_mode_ || i == 0) {
        // joint1 固定用 MIT 模式（硬件限制）
        motor_controller_->control_mit(*motors_[i], kp_, kd_, cmd_motor, target_vel, 0.0f);
      } else {
        motor_controller_->control_pos_vel(*motors_[i], cmd_motor, target_vel);
      }

      hw_commands_prev_[i] = cmd_motor / scale_to_motor;

    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DmHardwareInterface"),
        *rclcpp::Clock::make_shared(), 1000,
        "Write motor %s error: %s", joint_configs_[i].name.c_str(), e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────
//  return_zero_smooth：平滑插值归零
//  ROS1 等价：DMHardwareInterface::returnZero()
// ─────────────────────────────────────────────────────────
void DmHardwareInterface::return_zero_smooth()
{
  RCLCPP_INFO(rclcpp::get_logger("DmHardwareInterface"), "Returning to zero...");

  read(rclcpp::Time{}, rclcpp::Duration{0, 0});

  // 计算最大偏差，确定归零时间
  double max_diff = 0.0;
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    max_diff = std::max(max_diff, std::abs(hw_positions_[i]));
  }

  double duration = std::max(2.0, max_diff / 0.5);
  if (max_diff < 0.05) duration = 0.5;

  int steps = static_cast<int>(duration * control_frequency_);
  std::vector<double> start_pos = hw_positions_;

  for (int s = 0; s <= steps; ++s) {
    read(rclcpp::Time{}, rclcpp::Duration{0, 0});
    double t = static_cast<double>(s) / steps;
    double alpha = t * t * (3.0 - 2.0 * t);  // Smoothstep

    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      hw_commands_[i] = start_pos[i] * (1.0 - alpha);
    }

    write(rclcpp::Time{}, rclcpp::Duration::from_seconds(1.0 / control_frequency_));
    usleep(static_cast<useconds_t>(1000000.0 / control_frequency_));
  }

  // 保持零位 2.5 秒
  int hold = static_cast<int>(2.5 * control_frequency_);
  for (int i = 0; i < hold; ++i) {
    read(rclcpp::Time{}, rclcpp::Duration{0, 0});
    for (auto & c : hw_commands_) c = 0.0;
    write(rclcpp::Time{}, rclcpp::Duration::from_seconds(1.0 / control_frequency_));
    usleep(static_cast<useconds_t>(1000000.0 / control_frequency_));
  }
}

}  // namespace dm_arm_hardware
```

---

### 3.6 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(dm_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 共享库（pluginlib 插件必须是共享库）
add_library(dm_arm_hardware SHARED
  src/dm_hardware_interface.cpp
  src/SerialPort.cpp           # 直接复用 ROS1 版本
)

target_include_directories(dm_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_arm_hardware
  rclcpp
  hardware_interface
  pluginlib
)

# 导出插件描述
pluginlib_export_plugin_description_file(hardware_interface dm_arm_hardware.xml)

install(TARGETS dm_arm_hardware
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES dm_arm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

### 3.7 ros2_controllers.yaml 更新

Setup Assistant 生成的 `config/ros2_controllers.yaml` 需要对接真实硬件：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz，与硬件接口 control_frequency 保持一致

    # 声明要加载的控制器
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# arm_controller 详细配置
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    # 允许目标到达时有误差
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      joint1: {trajectory: 0.0, goal: 0.0}
      joint2: {trajectory: 0.0, goal: 0.0}
      joint3: {trajectory: 0.0, goal: 0.0}
      joint4: {trajectory: 0.0, goal: 0.0}
      joint5: {trajectory: 0.0, goal: 0.0}
      joint6: {trajectory: 0.0, goal: 0.0}

gripper_controller:
  ros__parameters:
    joints:
      - gripper_left
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    # 允许部分关节目标（因为 gripper_right 是 mimic，不在控制器里）
    allow_partial_joints_goal: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
      gripper_left: {trajectory: 0.0, goal: 0.0}
```

---

### 3.8 启动文件（带硬件接口）

创建 `dm_arm_moveit_config/launch/dm_arm_bringup.launch.py`：

```python
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                             TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 参数 ────────────────────────────────────────────────────
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
    use_rviz          = LaunchConfiguration('use_rviz',          default='true')

    # ── 机器人描述（URDF xacro，含 ros2_control 标签） ─────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('dm_arm_description'),
            'urdf', 'dm_arm_description.urdf.xacro'
        ]),
        ' use_fake_hardware:=', use_fake_hardware,
    ])
    robot_description = {'robot_description': robot_description_content}

    # ── SRDF ────────────────────────────────────────────────────
    robot_description_semantic_content = Command([
        'cat ',
        PathJoinSubstitution([
            FindPackageShare('dm_arm_moveit_config'),
            'config', 'dm_arm_description.srdf'
        ])
    ])
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # ── Kinematics ──────────────────────────────────────────────
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare('dm_arm_moveit_config'), 'config', 'kinematics.yaml'
    ])

    # ── ros2_controllers.yaml ───────────────────────────────────
    ros2_controllers = PathJoinSubstitution([
        FindPackageShare('dm_arm_moveit_config'), 'config', 'ros2_controllers.yaml'
    ])

    # ── controller_manager（硬件总线 + 控制器管理） ─────────────
    # 注意：robot_description 必须通过 parameters 传入，不能用话题
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers],
        output='screen',
    )

    # ── robot_state_publisher ───────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── 控制器启动（顺序依赖）──────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # arm_controller 在 joint_state_broadcaster 激活后再启动
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    # ── move_group ──────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            PathJoinSubstitution([
                FindPackageShare('dm_arm_moveit_config'), 'config', 'ompl_planning.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('dm_arm_moveit_config'), 'config', 'moveit_controllers.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('dm_arm_moveit_config'), 'config', 'joint_limits.yaml'
            ]),
            {'use_sim_time': False},
        ],
    )

    # ── RViz ────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('dm_arm_moveit_config'), 'config', 'moveit.rviz'
        ])],
        parameters=[robot_description, robot_description_semantic],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
            description='Use mock hardware (true) or real hardware (false)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,

        # move_group 需要 joint_states 已发布，延迟 2 秒
        TimerAction(period=2.0, actions=[move_group_node]),

        # RViz 在 move_group 之后启动
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
```

**验证硬件接口：**
```bash
# 构建
colcon build --symlink-install --packages-select dm_arm_hardware dm_arm_moveit_config

# 仿真模式（use_fake_hardware=true）
ros2 launch dm_arm_moveit_config dm_arm_bringup.launch.py use_fake_hardware:=true

# 验证控制器状态
ros2 control list_controllers
# 期望输出：
# arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# gripper_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# 验证 joint_states
ros2 topic hz /joint_states

# 真实硬件
ros2 launch dm_arm_moveit_config dm_arm_bringup.launch.py use_fake_hardware:=false
```

---

## 阶段四：运动规划核心

### 4.1 包结构

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake dm_arm_controller \
    --dependencies rclcpp moveit_ros_planning_interface \
    moveit_ros_move_group tf2_ros tf2_geometry_msgs geometry_msgs
```

```
dm_arm_controller/
├── CMakeLists.txt
├── package.xml
├── include/dm_arm_controller/
│   └── eef_cmd.hpp
└── src/
    └── eef_cmd.cpp
```

---

### 4.2 EefPoseCmd 头文件

`include/dm_arm_controller/eef_cmd.hpp`：

```cpp
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>

// MoveIt2 头文件（命名空间与 ROS1 相同，但构造函数需传入 node）
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>

// TF2（需传入 clock）
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

// ROS2 消息（命名空间加了 ::msg）
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rclcpp/rclcpp.hpp"

namespace dm_arm
{

// ── 任务动作类型（与 ROS1 完全相同） ──────────────────────────
enum class TargetAction_e { NONE = 0, PICK, STRETCH, ROTATE };

struct TaskTarget_t {
  geometry_msgs::msg::Pose pose;  // 注意：::msg:: 命名空间
  double wait_time = 0.0;
  TargetAction_e action = TargetAction_e::NONE;
  double param1 = 0.0;
};

// A* 搜索数据结构（与 ROS1 完全相同）
struct AStarNode_t { double droll, dpitch, g, h, f; };
struct PairHash_t {
  std::size_t operator()(const std::pair<int,int>& p) const noexcept {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};
struct AStarNodeCmper {
  bool operator()(const AStarNode_t& a, const AStarNode_t& b) const noexcept {
    return a.f > b.f;
  }
};

// ── EefPoseCmd ─────────────────────────────────────────────────
class EefPoseCmd
{
public:
  // ROS2：必须传入 rclcpp::Node::SharedPtr
  EefPoseCmd(const rclcpp::Node::SharedPtr & node, const std::string & plan_group_name);

  // 坐标系变换：末端 → 基座
  void eefTfBase(
    geometry_msgs::msg::PoseStamped & target_pose_eef,
    geometry_msgs::msg::PoseStamped & target_pose_base);

  // IK 有效性检查
  bool isIkValid(const geometry_msgs::msg::Pose & target_pose);

  // A* 搜索可达位姿
  bool searchReachablePose(
    geometry_msgs::msg::Pose & target_pose,
    double step, double radius);

  // 目标位姿设置与执行（基座坐标系）
  bool setGoalPoseBase(
    geometry_msgs::msg::PoseStamped & target_pose,
    bool allow_tweak = true,
    bool allow_feedforward = true);

  // 目标位姿设置与执行（末端坐标系）
  bool setGoalPoseEef(
    geometry_msgs::msg::PoseStamped & target_pose,
    bool allow_tweak = true,
    bool allow_feedforward = true);

  bool eefStretch(double distance);
  bool eefRotate(double angle_deg);  // 入参单位：度
  void resetToZero();

  geometry_msgs::msg::Pose getCurrentEefPose();
  std::vector<double> getCurrentJointPose();

private:
  rclcpp::Node::SharedPtr node_;

  // MoveIt2：构造需传入 node
  moveit::planning_interface::MoveGroupInterface arm_;

  // TF2：需传入 node->get_clock()
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // PlanningSceneMonitor：需传入 node
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;

  std::string plan_frame_;   // 规划参考系（base_link）
  std::string eef_frame_;    // 末端坐标系（link_tcp）

  const moveit::core::JointModelGroup * jmg_ = nullptr;
  moveit::core::RobotStatePtr current_state_;

  // 从参数读取
  double max_reach_;
  double min_reach_;
  int    max_iterations_;
};

// ── TaskGroupPlanner ───────────────────────────────────────────
class TaskGroupPlanner
{
public:
  explicit TaskGroupPlanner(EefPoseCmd & eef_cmd, const rclcpp::Node::SharedPtr & node);

  void add(const TaskTarget_t & target);
  void clear();
  void executeAll();

private:
  EefPoseCmd & eef_cmd_;
  rclcpp::Node::SharedPtr node_;
  std::vector<TaskTarget_t> task_list_;

  bool enable_optimization_;
  double pick_height_offset_;
  double place_height_offset_;
  double approach_distance_;
  double retreat_distance_;
  double default_wait_time_;
};

}  // namespace dm_arm
```

---

### 4.3 EefPoseCmd 实现

`src/eef_cmd.cpp`：

```cpp
#include "dm_arm_controller/eef_cmd.hpp"

#include <cmath>
#include <algorithm>

namespace dm_arm
{

// ─────────────────────────────────────────────────────────────────────────────
//  构造函数
//  ROS1：EefPoseCmd(const ros::NodeHandle& nh, const std::string& plan_group_name)
//  ROS2：EefPoseCmd(const rclcpp::Node::SharedPtr& node, const std::string& plan_group_name)
//
//  核心差异：
//  1. MoveGroupInterface(node, group)  ← 必须传 node
//  2. tf2_ros::Buffer(node->get_clock())  ← 必须传 clock
//  3. PlanningSceneMonitor(node, "robot_description")  ← 必须传 node
//  4. 参数读取：node->get_parameter() 而非 nh_.param()
// ─────────────────────────────────────────────────────────────────────────────
EefPoseCmd::EefPoseCmd(const rclcpp::Node::SharedPtr & node,
                       const std::string & plan_group_name)
: node_(node),
  arm_(node, plan_group_name),          // ← ROS2 关键差异
  tf_buffer_(node->get_clock()),        // ← ROS2 关键差异
  tf_listener_(tf_buffer_, node)        // ← ROS2 关键差异
{
  // 获取规划参考系与末端坐标系（API 与 ROS1 完全相同）
  plan_frame_ = arm_.getPlanningFrame();
  eef_frame_  = arm_.getEndEffectorLink();
  RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", plan_frame_.c_str());
  RCLCPP_INFO(node_->get_logger(), "EEF frame: %s",      eef_frame_.c_str());

  // ── 读取参数（ROS2 必须先 declare） ────────────────────────
  // 参数在 launch 文件或 yaml 中设置，通过 parameters= 传入节点
  node_->declare_parameter("moveit.allow_replanning",           true);
  node_->declare_parameter("end_effector.goal_position_tolerance",   0.015);
  node_->declare_parameter("end_effector.goal_orientation_tolerance", 0.05);
  node_->declare_parameter("end_effector.velocity_scaling",    0.3);
  node_->declare_parameter("end_effector.acceleration_scaling", 0.3);
  node_->declare_parameter("moveit.planner_id",                "RRTConnect");
  node_->declare_parameter("moveit.planning_time",             5.0);
  node_->declare_parameter("moveit.num_planning_attempts",     10);
  node_->declare_parameter("end_effector.max_reach",           0.6);
  node_->declare_parameter("end_effector.min_reach",           0.1);
  node_->declare_parameter("end_effector.search.step",         5.0);
  node_->declare_parameter("end_effector.search.radius",       45.0);
  node_->declare_parameter("end_effector.search.max_iterations", 100);

  bool allow_replanning        = node_->get_parameter("moveit.allow_replanning").as_bool();
  double goal_pos_tol          = node_->get_parameter("end_effector.goal_position_tolerance").as_double();
  double goal_ori_tol          = node_->get_parameter("end_effector.goal_orientation_tolerance").as_double();
  double vel_scale             = node_->get_parameter("end_effector.velocity_scaling").as_double();
  double acc_scale             = node_->get_parameter("end_effector.acceleration_scaling").as_double();
  std::string planner_id       = node_->get_parameter("moveit.planner_id").as_string();
  double planning_time         = node_->get_parameter("moveit.planning_time").as_double();
  int num_planning_attempts    = node_->get_parameter("moveit.num_planning_attempts").as_int();
  max_reach_                   = node_->get_parameter("end_effector.max_reach").as_double();
  min_reach_                   = node_->get_parameter("end_effector.min_reach").as_double();
  max_iterations_              = node_->get_parameter("end_effector.search.max_iterations").as_int();

  // ── 配置 MoveGroupInterface（API 与 ROS1 完全相同） ─────────
  arm_.allowReplanning(allow_replanning);
  arm_.setGoalPositionTolerance(goal_pos_tol);
  arm_.setGoalOrientationTolerance(goal_ori_tol);
  arm_.setMaxVelocityScalingFactor(vel_scale);
  arm_.setMaxAccelerationScalingFactor(acc_scale);
  arm_.setPlannerId(planner_id);
  arm_.setPlanningTime(planning_time);
  arm_.setNumPlanningAttempts(num_planning_attempts);

  // ── PlanningSceneMonitor（需传入 node） ─────────────────────
  scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node_, "robot_description");                    // ← ROS2 需传 node
  scene_monitor_->startSceneMonitor("/planning_scene");
  scene_monitor_->startWorldGeometryMonitor();
  scene_monitor_->startStateMonitor("/joint_states");

  // 等待场景加载
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 获取 JointModelGroup 指针（用于 IK 计算）
  // 必须等 getCurrentState() 返回非空
  current_state_ = arm_.getCurrentState(2.0);
  if (!current_state_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current state");
    throw std::runtime_error("Failed to get current state");
  }
  jmg_ = current_state_->getJointModelGroup(plan_group_name);
}

// ─────────────────────────────────────────────────────────────────────────────
//  eefTfBase：末端坐标系 → 基座坐标系变换
//  ROS1：tf2::doTransform(target_pose_eef, target_pose_base, tf_stamped)
//  ROS2：API 完全相同，但 Buffer 构造需 clock
// ─────────────────────────────────────────────────────────────────────────────
void EefPoseCmd::eefTfBase(
  geometry_msgs::msg::PoseStamped & target_pose_eef,
  geometry_msgs::msg::PoseStamped & target_pose_base)
{
  // lookupTransform(target_frame, source_frame, time, timeout)
  // ROS1：ros::Time(0) → ROS2：tf2::TimePointZero
  geometry_msgs::msg::TransformStamped tf_stamped =
    tf_buffer_.lookupTransform(plan_frame_, eef_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

  tf2::doTransform(target_pose_eef, target_pose_base, tf_stamped);
}

// ─────────────────────────────────────────────────────────────────────────────
//  isIkValid：检查位姿是否有逆运动学解
//  API 与 ROS1 完全相同：setFromIK(jmg, pose, timeout)
// ─────────────────────────────────────────────────────────────────────────────
bool EefPoseCmd::isIkValid(const geometry_msgs::msg::Pose & target_pose)
{
  // setFromIK 是 RobotState 的方法，不依赖 ROS 版本
  // 返回 true 表示找到 IK 解（内部已设置关节值）
  return current_state_->setFromIK(jmg_, target_pose, 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  searchReachablePose：A* 搜索最近可达姿态
//  逻辑与 ROS1 完全相同，消息类型改为 ::msg::
// ─────────────────────────────────────────────────────────────────────────────
bool EefPoseCmd::searchReachablePose(
  geometry_msgs::msg::Pose & target_pose,
  double step, double radius)
{
  if (isIkValid(target_pose)) return true;

  // 判断是否 RPY=0（需要前馈计算）
  bool need_feedforward = (
    std::abs(target_pose.orientation.w - 1.0) < 1e-8 &&
    std::abs(target_pose.orientation.x)        < 1e-8 &&
    std::abs(target_pose.orientation.y)        < 1e-8 &&
    std::abs(target_pose.orientation.z)        < 1e-8);

  // 获取末端坐标系下的位姿
  geometry_msgs::msg::Pose target_pose_eef = target_pose;
  geometry_msgs::msg::TransformStamped tf_to_eef =
    tf_buffer_.lookupTransform(eef_frame_, plan_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  geometry_msgs::msg::TransformStamped tf_to_base =
    tf_buffer_.lookupTransform(plan_frame_, eef_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  tf2::doTransform(target_pose_eef, target_pose_eef, tf_to_eef);

  step   = step   * M_PI / 180.0;
  radius = radius * M_PI / 180.0;

  // 前馈：末端 Z 轴对准从基座到目标点的方向
  tf2::Quaternion q_orig;
  double roll_orig, pitch_orig, yaw_orig;

  if (need_feedforward) {
    tf2::Vector3 z_axis(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    if (z_axis.length() < 1e-8) { RCLCPP_ERROR(node_->get_logger(), "Invalid target position"); return false; }
    z_axis.normalize();

    tf2::Vector3 x_axis(1, 0, 0);
    if (std::abs(z_axis.dot(x_axis)) > 0.9999) x_axis = tf2::Vector3(0, 1, 0);
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalize();

    tf2::Matrix3x3 rot(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion q_feed_base;
    rot.getRotation(q_feed_base);

    // 转换到末端坐标系
    geometry_msgs::msg::Quaternion q_feed_eef_msg = tf2::toMsg(q_feed_base);
    tf2::doTransform(q_feed_eef_msg, q_feed_eef_msg, tf_to_eef);
    target_pose_eef.orientation = q_feed_eef_msg;
  }

  tf2::fromMsg(target_pose_eef.orientation, q_orig);
  tf2::Matrix3x3(q_orig).getRPY(roll_orig, pitch_orig, yaw_orig);

  // ── A* 搜索 ─────────────────────────────────────────────────
  const int step_count = static_cast<int>(std::ceil(radius / step));
  auto heuristic = [](double dr, double dp) { return std::hypot(dr, dp); };

  std::priority_queue<AStarNode_t, std::vector<AStarNode_t>, AStarNodeCmper> open_set;
  open_set.push({0.0, 0.0, 0.0, heuristic(0.0, 0.0), 0.0});

  using Key = std::pair<int, int>;
  std::unordered_map<Key, double, PairHash_t> closed_set;
  closed_set[{0, 0}] = 0.0;

  const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1}};

  size_t max_expand = max_iterations_ > 0
    ? static_cast<size_t>(max_iterations_)
    : static_cast<size_t>((2*step_count+1) * (2*step_count+1) * 10);

  size_t expand_count = 0;

  while (!open_set.empty()) {
    if (++expand_count > max_expand) {
      RCLCPP_ERROR(node_->get_logger(), "A* exceeded max expansions (%zu)", max_expand);
      break;
    }

    AStarNode_t cur = open_set.top(); open_set.pop();

    // 候选位姿
    tf2::Quaternion q_cand;
    q_cand.setRPY(roll_orig + cur.droll, pitch_orig + cur.dpitch, 0.0);

    geometry_msgs::msg::Pose pose_cand = target_pose_eef;
    pose_cand.orientation = tf2::toMsg(q_cand);

    // 转回基座坐标系，检查 IK + 碰撞
    geometry_msgs::msg::Pose pose_cand_base;
    tf2::doTransform(pose_cand, pose_cand_base, tf_to_base);

    if (isIkValid(pose_cand_base)) {
      // 获取场景的读锁来检测碰撞
      {
        planning_scene_monitor::LockedPlanningSceneRO scene(scene_monitor_);
        if (scene->isStateColliding(*current_state_, jmg_->getName())) {
          RCLCPP_WARN(node_->get_logger(), "Candidate pose colliding, continue searching");
          continue;
        }
      }

      RCLCPP_INFO(node_->get_logger(),
        "Found reachable pose: droll=%.1f°, dpitch=%.1f°",
        cur.droll * 180.0 / M_PI, cur.dpitch * 180.0 / M_PI);
      target_pose = pose_cand_base;
      return true;
    }

    // 展开邻居
    int cr = static_cast<int>(std::round(cur.droll / step));
    int cp = static_cast<int>(std::round(cur.dpitch / step));

    for (auto & d : dirs) {
      int nr = cr + d[0], np = cp + d[1];
      double ndr = nr * step, ndp = np * step;
      if (std::hypot(ndr, ndp) > radius + 1e-12) continue;

      Key key{nr, np};
      double move_cost = std::hypot(ndr - cur.droll, ndp - cur.dpitch);
      double ng = cur.g + move_cost;

      auto it = closed_set.find(key);
      if (it != closed_set.end() && it->second <= ng) continue;
      closed_set[key] = ng;
      double nh = heuristic(ndr, ndp);
      open_set.push({ndr, ndp, ng, nh, ng + nh});
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "No reachable pose found");
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  setGoalPoseBase：主规划执行函数（基座坐标系）
//  流程：刷新当前状态 → 搜索可达位姿 → setPoseTarget → plan → execute
//  ROS1：ros::Time::now() → ROS2：node_->now()
// ─────────────────────────────────────────────────────────────────────────────
bool EefPoseCmd::setGoalPoseBase(
  geometry_msgs::msg::PoseStamped & target_pose,
  bool allow_tweak, bool allow_feedforward)
{
  target_pose.header.frame_id = plan_frame_;
  target_pose.header.stamp    = node_->now();  // ← ROS2 差异

  // 刷新当前状态（ROS2 需指定超时秒数）
  arm_.setStartStateToCurrentState();
  current_state_ = arm_.getCurrentState(2.0);  // ← ROS2 需指定超时
  if (!current_state_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current state");
    return false;
  }

  // 可达性搜索
  if (allow_tweak) {
    if (allow_feedforward) {
      // 前馈：清零姿态，让 searchReachablePose 自动计算最优方向
      target_pose.pose.orientation.w = 1.0;
      target_pose.pose.orientation.x = 0.0;
      target_pose.pose.orientation.y = 0.0;
      target_pose.pose.orientation.z = 0.0;
    }

    double step   = node_->get_parameter("end_effector.search.step").as_double();
    double radius = node_->get_parameter("end_effector.search.radius").as_double();

    geometry_msgs::msg::Pose pose_candidate = target_pose.pose;
    if (!searchReachablePose(pose_candidate, step, radius)) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot find reachable pose");
      return false;
    }
    target_pose.pose = pose_candidate;
  } else {
    if (!isIkValid(target_pose.pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Target pose has no IK solution");
      return false;
    }
  }

  // 规划
  arm_.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // ROS2：plan() 返回 moveit::core::MoveItErrorCode（与 ROS1 相同）
  bool success = (arm_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    return false;
  }

  // 执行
  success = (arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Motion succeeded");
  return true;
}

bool EefPoseCmd::setGoalPoseEef(
  geometry_msgs::msg::PoseStamped & target_pose,
  bool allow_tweak, bool allow_feedforward)
{
  geometry_msgs::msg::PoseStamped target_pose_base;
  eefTfBase(target_pose, target_pose_base);
  return setGoalPoseBase(target_pose_base, allow_tweak, allow_feedforward);
}

// ─────────────────────────────────────────────────────────────────────────────
//  eefStretch：末端沿自身 Z 轴前伸/后缩
// ─────────────────────────────────────────────────────────────────────────────
bool EefPoseCmd::eefStretch(double distance)
{
  geometry_msgs::msg::PoseStamped target;
  target.pose.position.z   = distance;
  target.pose.orientation.w = 1.0;
  return setGoalPoseEef(target, true, false);
}

// ─────────────────────────────────────────────────────────────────────────────
//  eefRotate：末端绕自身 Z 轴旋转（单位：度）
// ─────────────────────────────────────────────────────────────────────────────
bool EefPoseCmd::eefRotate(double angle_deg)
{
  double angle_rad = angle_deg * M_PI / 180.0;

  geometry_msgs::msg::Pose current = getCurrentEefPose();

  tf2::Quaternion q_cur;
  tf2::fromMsg(current.orientation, q_cur);
  tf2::Quaternion q_delta;
  q_delta.setRPY(0, 0, angle_rad);
  tf2::Quaternion q_new = q_cur * q_delta;

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = plan_frame_;
  target.pose = current;
  target.pose.orientation = tf2::toMsg(q_new);

  return setGoalPoseBase(target, true, false);
}

// ─────────────────────────────────────────────────────────────────────────────
//  resetToZero：移动到 SRDF 定义的 "zero" 命名姿态
//  API 与 ROS1 完全相同
// ─────────────────────────────────────────────────────────────────────────────
void EefPoseCmd::resetToZero()
{
  RCLCPP_INFO(node_->get_logger(), "Resetting to zero pose");
  arm_.setNamedTarget("zero");
  arm_.move();
}

geometry_msgs::msg::Pose EefPoseCmd::getCurrentEefPose()
{
  return arm_.getCurrentPose().pose;
}

std::vector<double> EefPoseCmd::getCurrentJointPose()
{
  return arm_.getCurrentJointValues();
}

// ─────────────────────────────────────────────────────────────────────────────
//  TaskGroupPlanner
// ─────────────────────────────────────────────────────────────────────────────
TaskGroupPlanner::TaskGroupPlanner(EefPoseCmd & eef_cmd, const rclcpp::Node::SharedPtr & node)
: eef_cmd_(eef_cmd), node_(node)
{
  node_->declare_parameter("task_planner.enable_optimization",  true);
  node_->declare_parameter("task_planner.pick_height_offset",   0.1);
  node_->declare_parameter("task_planner.place_height_offset",  0.1);
  node_->declare_parameter("task_planner.approach_distance",    0.05);
  node_->declare_parameter("task_planner.retreat_distance",     0.05);
  node_->declare_parameter("task_planner.default_wait_time",    0.5);

  enable_optimization_ = node_->get_parameter("task_planner.enable_optimization").as_bool();
  pick_height_offset_  = node_->get_parameter("task_planner.pick_height_offset").as_double();
  default_wait_time_   = node_->get_parameter("task_planner.default_wait_time").as_double();
}

void TaskGroupPlanner::add(const TaskTarget_t & target) { task_list_.push_back(target); }
void TaskGroupPlanner::clear() { task_list_.clear(); }

// ─────────────────────────────────────────────────────────────────────────────
//  executeAll：贪心最近邻排序后依次执行
// ─────────────────────────────────────────────────────────────────────────────
void TaskGroupPlanner::executeAll()
{
  if (task_list_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Task list is empty");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Executing %zu tasks", task_list_.size());

  // 贪心最近邻排序
  std::vector<TaskTarget_t> sorted, pending = task_list_;
  geometry_msgs::msg::Pose cur_pose = eef_cmd_.getCurrentEefPose();

  auto dist = [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  };

  while (!pending.empty()) {
    auto it = std::min_element(pending.begin(), pending.end(),
      [&](const auto & a, const auto & b) { return dist(cur_pose, a.pose) < dist(cur_pose, b.pose); });
    sorted.push_back(*it);
    cur_pose = it->pose;
    pending.erase(it);
  }

  // 依次执行
  size_t idx = 1;
  for (const auto & task : sorted) {
    RCLCPP_INFO(node_->get_logger(), "Task [%zu/%zu] starting", idx, sorted.size());

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.pose = task.pose;
    bool ok = eef_cmd_.setGoalPoseBase(target_pose);
    if (!ok) { RCLCPP_WARN(node_->get_logger(), "Task [%zu] move failed", idx); }

    if (ok && task.action == TargetAction_e::STRETCH) {
      ok = eef_cmd_.eefStretch(task.param1);
      if (!ok) RCLCPP_WARN(node_->get_logger(), "Task [%zu] stretch failed", idx);
    } else if (ok && task.action == TargetAction_e::ROTATE) {
      ok = eef_cmd_.eefRotate(task.param1);
      if (!ok) RCLCPP_WARN(node_->get_logger(), "Task [%zu] rotate failed", idx);
    }

    double wait = (task.wait_time > 0.0) ? task.wait_time : default_wait_time_;
    if (wait > 0.0) rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(wait)));

    ++idx;
  }

  RCLCPP_INFO(node_->get_logger(), "All tasks completed");
  clear();
}

}  // namespace dm_arm
```

---

## 阶段五：障碍物管理

### 5.1 PlanningSceneInterface 基础用法

ROS2 中 `PlanningSceneInterface` 与 ROS1 用法基本相同，但需传入 `node` 的命名空间：

```cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// ROS2：可选传入 node 命名空间（通常不需要）
moveit::planning_interface::PlanningSceneInterface scene;

// ── 添加立方体障碍物 ──────────────────────────────────────────
moveit_msgs::msg::CollisionObject box;
box.id = "table";
box.header.frame_id = "base_link";

shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.6;   // x（宽）
primitive.dimensions[1] = 1.2;   // y（长）
primitive.dimensions[2] = 0.02;  // z（高）

geometry_msgs::msg::Pose pose;
pose.position.x = 0.5;
pose.position.y = 0.0;
pose.position.z = -0.01;
pose.orientation.w = 1.0;

box.primitives.push_back(primitive);
box.primitive_poses.push_back(pose);
box.operation = moveit_msgs::msg::CollisionObject::ADD;

scene.applyCollisionObject(box);

// ── 添加柱体障碍物 ────────────────────────────────────────────
moveit_msgs::msg::CollisionObject cylinder;
cylinder.id = "post";
cylinder.header.frame_id = "base_link";

shape_msgs::msg::SolidPrimitive cyl_shape;
cyl_shape.type = cyl_shape.CYLINDER;
cyl_shape.dimensions.resize(2);
cyl_shape.dimensions[0] = 1.0;   // 高度（Z）
cyl_shape.dimensions[1] = 0.05;  // 半径

geometry_msgs::msg::Pose cyl_pose;
cyl_pose.position.x = 0.3;
cyl_pose.position.y = 0.3;
cyl_pose.position.z = 0.5;
cyl_pose.orientation.w = 1.0;

cylinder.primitives.push_back(cyl_shape);
cylinder.primitive_poses.push_back(cyl_pose);
cylinder.operation = moveit_msgs::msg::CollisionObject::ADD;

scene.applyCollisionObject(cylinder);

// ── 批量添加 ──────────────────────────────────────────────────
std::vector<moveit_msgs::msg::CollisionObject> objects = {box, cylinder};
scene.applyCollisionObjects(objects);

// ── 移除障碍物 ────────────────────────────────────────────────
scene.removeCollisionObjects({"table", "post"});

// ── 附着物体（抓取后） ────────────────────────────────────────
// 将场景中的物体附着到末端，规划时不会把该物体当作障碍物
arm_.attachObject("target_box", "link_tcp", {"link_tcp", "finger_left", "finger_right"});

// ── 分离物体（放置后） ────────────────────────────────────────
arm_.detachObject("target_box");
// 分离后物体回到规划场景，再次成为碰撞检测对象
```

### 5.2 在 EefPoseCmd 中封装障碍物管理

在 `eef_cmd.hpp` 中增加 `Barrier` 类：

```cpp
class Barrier
{
public:
  explicit Barrier(const rclcpp::Node::SharedPtr & node);

  // 添加/更新障碍物（支持 BOX、CYLINDER、SPHERE）
  void addBox(const std::string & id,
              double size_x, double size_y, double size_z,
              double px, double py, double pz,
              const std::string & frame_id = "base_link");

  void addCylinder(const std::string & id,
                   double height, double radius,
                   double px, double py, double pz,
                   const std::string & frame_id = "base_link");

  void remove(const std::string & id);
  void removeAll();

  // 获取当前场景中所有碰撞对象 ID
  std::vector<std::string> listObjects();

private:
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::Node::SharedPtr node_;

  moveit_msgs::msg::CollisionObject makeObject(
    const std::string & id, const std::string & frame_id,
    const shape_msgs::msg::SolidPrimitive & shape,
    const geometry_msgs::msg::Pose & pose);
};
```

`src/barrier.cpp`：

```cpp
#include "dm_arm_controller/eef_cmd.hpp"

namespace dm_arm
{

Barrier::Barrier(const rclcpp::Node::SharedPtr & node) : node_(node) {}

void Barrier::addBox(const std::string & id,
                     double sx, double sy, double sz,
                     double px, double py, double pz,
                     const std::string & frame_id)
{
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape.BOX;
  shape.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose;
  pose.position.x = px; pose.position.y = py; pose.position.z = pz;
  pose.orientation.w = 1.0;

  scene_.applyCollisionObject(makeObject(id, frame_id, shape, pose));
  RCLCPP_INFO(node_->get_logger(), "Added box '%s' at (%.2f, %.2f, %.2f)", id.c_str(), px, py, pz);
}

void Barrier::addCylinder(const std::string & id,
                           double height, double radius,
                           double px, double py, double pz,
                           const std::string & frame_id)
{
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape.CYLINDER;
  shape.dimensions = {height, radius};

  geometry_msgs::msg::Pose pose;
  pose.position.x = px; pose.position.y = py; pose.position.z = pz;
  pose.orientation.w = 1.0;

  scene_.applyCollisionObject(makeObject(id, frame_id, shape, pose));
  RCLCPP_INFO(node_->get_logger(), "Added cylinder '%s' at (%.2f, %.2f, %.2f)", id.c_str(), px, py, pz);
}

void Barrier::remove(const std::string & id) { scene_.removeCollisionObjects({id}); }
void Barrier::removeAll() { scene_.removeCollisionObjects(listObjects()); }

std::vector<std::string> Barrier::listObjects()
{
  auto objects = scene_.getObjects();
  std::vector<std::string> ids;
  ids.reserve(objects.size());
  for (const auto & p : objects) ids.push_back(p.first);
  return ids;
}

moveit_msgs::msg::CollisionObject Barrier::makeObject(
  const std::string & id, const std::string & frame_id,
  const shape_msgs::msg::SolidPrimitive & shape,
  const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;
  obj.primitives.push_back(shape);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

}  // namespace dm_arm
```

---

## 阶段六：服务层封装

### 6.1 自定义服务消息

创建 `dm_arm_msgs` 包：

```bash
ros2 pkg create --build-type ament_cmake dm_arm_msgs --dependencies rosidl_default_generators
```

`srv/DmArmCmd.srv`（ROS2 服务文件命名必须 CamelCase）：

```
# 请求
string command
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
string param1
string param2
string param3
float64[] joints
---
# 响应
bool success
string message
float64 cur_x
float64 cur_y
float64 cur_z
float64 cur_roll
float64 cur_pitch
float64 cur_yaw
float64[] cur_joints
```

`CMakeLists.txt`：

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DmArmCmd.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
```

---

### 6.2 服务端实现（dm_arm_server 包）

`include/dm_arm_server/dm_arm_server.hpp`：

```cpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "dm_arm_controller/eef_cmd.hpp"
#include "dm_arm_msgs/srv/dm_arm_cmd.hpp"

namespace dm_arm
{

class DmArmServer : public rclcpp::Node
{
public:
  explicit DmArmServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 服务回调（ROS2：参数为 shared_ptr）
  void eefCmdCallback(
    const std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Request> request,
    std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Response> response);

  void taskPlannerCallback(
    const std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Request> request,
    std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Response> response);

  // MoveIt2 接口（必须在节点构造后初始化）
  std::shared_ptr<EefPoseCmd>       eef_controller_;
  std::shared_ptr<TaskGroupPlanner> task_planner_;
  std::shared_ptr<Barrier>          barrier_;

  // ROS2 服务（类型改为 rclcpp::Service）
  rclcpp::Service<dm_arm_msgs::srv::DmArmCmd>::SharedPtr srv_eef_cmd_;
  rclcpp::Service<dm_arm_msgs::srv::DmArmCmd>::SharedPtr srv_task_planner_;
};

}  // namespace dm_arm
```

`src/dm_arm_server.cpp`：

```cpp
#include "dm_arm_server/dm_arm_server.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dm_arm
{

DmArmServer::DmArmServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("dm_arm_server", options)
{
  // ── 声明参数 ────────────────────────────────────────────────
  this->declare_parameter("moveit.planning_group", "arm");

  std::string planning_group = this->get_parameter("moveit.planning_group").as_string();

  // ── 初始化 MoveIt2 接口 ─────────────────────────────────────
  // 注意：MoveGroupInterface 的构造会阻塞直到 move_group 节点可用
  // 因此 DmArmServer 节点启动前必须等待 move_group 启动
  eef_controller_ = std::make_shared<EefPoseCmd>(
    this->shared_from_this(),  // ← 传入 shared_ptr
    planning_group);

  task_planner_ = std::make_shared<TaskGroupPlanner>(
    *eef_controller_,
    this->shared_from_this());

  barrier_ = std::make_shared<Barrier>(this->shared_from_this());

  // ── 创建服务 ────────────────────────────────────────────────
  // ROS1：nh.advertiseService("name", callback, this)
  // ROS2：create_service<SrvType>("name", callback)
  srv_eef_cmd_ = this->create_service<dm_arm_msgs::srv::DmArmCmd>(
    "/dm_arm_server/eef_cmd",
    std::bind(&DmArmServer::eefCmdCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_task_planner_ = this->create_service<dm_arm_msgs::srv::DmArmCmd>(
    "/dm_arm_server/task_planner",
    std::bind(&DmArmServer::taskPlannerCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // 回零
  eef_controller_->resetToZero();

  RCLCPP_INFO(this->get_logger(), "DmArmServer ready");
  RCLCPP_INFO(this->get_logger(), "  /dm_arm_server/eef_cmd");
  RCLCPP_INFO(this->get_logger(), "  /dm_arm_server/task_planner");
}

void DmArmServer::eefCmdCallback(
  const std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Request> req,
  std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Response> res)
{
  // ── zero ────────────────────────────────────────────────────
  if (req->command == "zero") {
    eef_controller_->resetToZero();
    res->success = true;
    res->message = "回到零点成功";

  // ── goal_base / goal_eef ─────────────────────────────────────
  } else if (req->command == "goal_base" || req->command == "goal_eef") {
    geometry_msgs::msg::PoseStamped target;
    tf2::Quaternion qtn;
    qtn.setRPY(req->roll, req->pitch, req->yaw);
    qtn.normalize();

    target.pose.position.x    = req->x;
    target.pose.position.y    = req->y;
    target.pose.position.z    = req->z;
    target.pose.orientation   = tf2::toMsg(qtn);

    bool ok;
    if (req->command == "goal_base") {
      ok = eef_controller_->setGoalPoseBase(target, true, true);
    } else {
      ok = eef_controller_->setGoalPoseEef(target, true, true);
    }

    res->success = ok;
    res->message = ok ? "设置目标位姿成功" : "设置目标位姿失败";

  // ── stretch ──────────────────────────────────────────────────
  } else if (req->command == "stretch") {
    bool ok = eef_controller_->eefStretch(std::stod(req->param1));
    res->success = ok;
    res->message = ok ? "末端伸缩成功" : "末端伸缩失败";

  // ── rotate ───────────────────────────────────────────────────
  } else if (req->command == "rotate") {
    bool ok = eef_controller_->eefRotate(std::stod(req->param1));
    res->success = ok;
    res->message = ok ? "末端旋转成功" : "末端旋转失败";

  // ── get_pose ─────────────────────────────────────────────────
  } else if (req->command == "get_pose") {
    geometry_msgs::msg::Pose cur = eef_controller_->getCurrentEefPose();
    tf2::Quaternion qtn;
    tf2::fromMsg(cur.orientation, qtn);
    double roll, pitch, yaw;
    tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

    res->cur_x = cur.position.x; res->cur_y = cur.position.y; res->cur_z = cur.position.z;
    res->cur_roll = roll; res->cur_pitch = pitch; res->cur_yaw = yaw;
    res->success = true;
    res->message = "获取末端位姿成功";

  // ── get_joints ───────────────────────────────────────────────
  } else if (req->command == "get_joints") {
    res->cur_joints = eef_controller_->getCurrentJointPose();
    res->success = true;
    res->message = "获取关节角度成功";

  // ── add_barrier / remove_barrier ─────────────────────────────
  } else if (req->command == "add_box") {
    // param1: "id", param2: "sx sy sz", param3: "px py pz"
    // 实际项目可设计更细致的参数格式
    barrier_->addBox(req->param1,
      std::stod(req->param2), std::stod(req->param3), 0.02,
      req->x, req->y, req->z);
    res->success = true;
    res->message = "添加障碍物成功";

  } else if (req->command == "remove_barrier") {
    barrier_->remove(req->param1);
    res->success = true;
    res->message = "移除障碍物成功";

  } else {
    res->success = false;
    res->message = "未知命令: " + req->command;
  }
}

void DmArmServer::taskPlannerCallback(
  const std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Request> req,
  std::shared_ptr<dm_arm_msgs::srv::DmArmCmd::Response> res)
{
  if (req->command == "add_task") {
    TaskTarget_t target;
    tf2::Quaternion qtn;
    qtn.setRPY(req->roll, req->pitch, req->yaw);
    qtn.normalize();
    target.pose.position.x   = req->x;
    target.pose.position.y   = req->y;
    target.pose.position.z   = req->z;
    target.pose.orientation  = tf2::toMsg(qtn);
    target.wait_time = req->param1.empty() ? 0.0 : std::stod(req->param1);

    if      (req->param2 == "PICK")    target.action = TargetAction_e::PICK;
    else if (req->param2 == "STRETCH") target.action = TargetAction_e::STRETCH;
    else if (req->param2 == "ROTATE")  target.action = TargetAction_e::ROTATE;
    else                               target.action = TargetAction_e::NONE;

    target.param1 = req->param3.empty() ? 0.0 : std::stod(req->param3);
    task_planner_->add(target);
    res->success = true; res->message = "添加任务成功";

  } else if (req->command == "clear_tasks") {
    task_planner_->clear();
    res->success = true; res->message = "清除任务成功";

  } else if (req->command == "exe_all_tasks") {
    task_planner_->executeAll();
    res->success = true; res->message = "开始执行所有任务";

  } else {
    res->success = false; res->message = "未知命令";
  }
}

}  // namespace dm_arm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 使用 MultiThreadedExecutor 以支持服务并发处理
  // ROS1 使用 AsyncSpinner(n)，ROS2 使用 MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;

  auto server_node = std::make_shared<dm_arm::DmArmServer>();
  executor.add_node(server_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "DmArmServer spinning");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
```

---

### 6.3 客户端封装

`include/dm_arm_server/dm_arm_client.hpp`：

```cpp
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "dm_arm_msgs/srv/dm_arm_cmd.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace dm_arm
{

class DmArmClient
{
public:
  explicit DmArmClient(const rclcpp::Node::SharedPtr & node);

  // 基础动作
  bool zero(std::string * msg = nullptr);
  bool goalBase(double x, double y, double z,
                double roll, double pitch, double yaw,
                std::string * msg = nullptr);
  bool goalEef(double x, double y, double z,
               double roll, double pitch, double yaw,
               std::string * msg = nullptr);
  bool stretch(double distance, std::string * msg = nullptr);
  bool rotate(double angle_deg, std::string * msg = nullptr);

  // 查询
  bool getPose(double & x, double & y, double & z,
               double & roll, double & pitch, double & yaw);
  bool getJoints(std::vector<double> & joints);

  // 任务规划
  bool addTask(const geometry_msgs::msg::Pose & pose,
               double wait_time, const std::string & action, double param1 = 0.0);
  bool clearTasks();
  bool executeAllTasks();

private:
  // ROS2：返回值是 std::shared_future，需要 spin_until_future_complete
  bool sendCmd(
    const dm_arm_msgs::srv::DmArmCmd::Request & req,
    dm_arm_msgs::srv::DmArmCmd::Response & res);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<dm_arm_msgs::srv::DmArmCmd>::SharedPtr client_eef_;
  rclcpp::Client<dm_arm_msgs::srv::DmArmCmd>::SharedPtr client_task_;
};

}  // namespace dm_arm
```

`src/dm_arm_client.cpp`：

```cpp
#include "dm_arm_server/dm_arm_client.hpp"

namespace dm_arm
{

DmArmClient::DmArmClient(const rclcpp::Node::SharedPtr & node) : node_(node)
{
  // ROS1：nh.serviceClient<T>("name")
  // ROS2：node->create_client<T>("name")
  client_eef_  = node_->create_client<dm_arm_msgs::srv::DmArmCmd>("/dm_arm_server/eef_cmd");
  client_task_ = node_->create_client<dm_arm_msgs::srv::DmArmCmd>("/dm_arm_server/task_planner");

  // 等待服务可用
  RCLCPP_INFO(node_->get_logger(), "Waiting for dm_arm_server services...");
  client_eef_->wait_for_service(std::chrono::seconds(30));
  client_task_->wait_for_service(std::chrono::seconds(30));
  RCLCPP_INFO(node_->get_logger(), "Services ready");
}

bool DmArmClient::sendCmd(
  const dm_arm_msgs::srv::DmArmCmd::Request & req,
  dm_arm_msgs::srv::DmArmCmd::Response & res)
{
  auto request = std::make_shared<dm_arm_msgs::srv::DmArmCmd::Request>(req);

  // 根据 command 决定调用哪个服务
  rclcpp::Client<dm_arm_msgs::srv::DmArmCmd>::SharedPtr client;
  if (req.command == "add_task" || req.command == "clear_tasks" ||
      req.command == "exe_all_tasks") {
    client = client_task_;
  } else {
    client = client_eef_;
  }

  // ROS2 异步调用：返回 future
  auto future = client->async_send_request(request);

  // 等待结果（spin_until_future_complete 会临时驱动 executor）
  // 注意：如果在同一节点的回调中调用，会死锁！需要在独立线程或 executor 外调用
  if (rclcpp::spin_until_future_complete(node_, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    res = *future.get();
    if (!res.success) {
      RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", res.message.c_str());
    }
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Service call timeout or error");
  return false;
}

bool DmArmClient::zero(std::string * msg)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "zero";
  bool ok = sendCmd(req, res);
  if (msg) *msg = res.message;
  return ok && res.success;
}

bool DmArmClient::goalBase(double x, double y, double z,
                            double roll, double pitch, double yaw,
                            std::string * msg)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "goal_base";
  req.x = x; req.y = y; req.z = z;
  req.roll = roll; req.pitch = pitch; req.yaw = yaw;
  bool ok = sendCmd(req, res);
  if (msg) *msg = res.message;
  return ok && res.success;
}

bool DmArmClient::goalEef(double x, double y, double z,
                           double roll, double pitch, double yaw,
                           std::string * msg)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "goal_eef";
  req.x = x; req.y = y; req.z = z;
  req.roll = roll; req.pitch = pitch; req.yaw = yaw;
  bool ok = sendCmd(req, res);
  if (msg) *msg = res.message;
  return ok && res.success;
}

bool DmArmClient::stretch(double distance, std::string * msg)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "stretch";
  req.param1 = std::to_string(distance);
  bool ok = sendCmd(req, res);
  if (msg) *msg = res.message;
  return ok && res.success;
}

bool DmArmClient::rotate(double angle_deg, std::string * msg)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "rotate";
  req.param1 = std::to_string(angle_deg);
  bool ok = sendCmd(req, res);
  if (msg) *msg = res.message;
  return ok && res.success;
}

bool DmArmClient::getPose(double & x, double & y, double & z,
                           double & roll, double & pitch, double & yaw)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "get_pose";
  if (!sendCmd(req, res) || !res.success) return false;
  x = res.cur_x; y = res.cur_y; z = res.cur_z;
  roll = res.cur_roll; pitch = res.cur_pitch; yaw = res.cur_yaw;
  return true;
}

bool DmArmClient::getJoints(std::vector<double> & joints)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "get_joints";
  if (!sendCmd(req, res) || !res.success) return false;
  joints = res.cur_joints;
  return true;
}

bool DmArmClient::addTask(const geometry_msgs::msg::Pose & pose,
                           double wait_time, const std::string & action, double param1)
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "add_task";
  req.x = pose.position.x; req.y = pose.position.y; req.z = pose.position.z;
  // 从四元数提取 RPY
  tf2::Quaternion q; tf2::fromMsg(pose.orientation, q);
  double r, p, y_ang; tf2::Matrix3x3(q).getRPY(r, p, y_ang);
  req.roll = r; req.pitch = p; req.yaw = y_ang;
  req.param1 = std::to_string(wait_time);
  req.param2 = action;
  req.param3 = std::to_string(param1);
  return sendCmd(req, res) && res.success;
}

bool DmArmClient::clearTasks()
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "clear_tasks";
  return sendCmd(req, res) && res.success;
}

bool DmArmClient::executeAllTasks()
{
  dm_arm_msgs::srv::DmArmCmd::Request req;
  dm_arm_msgs::srv::DmArmCmd::Response res;
  req.command = "exe_all_tasks";
  return sendCmd(req, res) && res.success;
}

}  // namespace dm_arm
```

---

### 6.4 验证命令

```bash
# 构建全部包
colcon build --symlink-install

# 启动（仿真模式）
ros2 launch dm_arm_moveit_config dm_arm_bringup.launch.py use_fake_hardware:=true

# 另开终端：启动服务器
ros2 run dm_arm_server dm_arm_server --ros-args --params-file ~/ros2_ws/src/dm_arm_server/config/dm_arm_config.yaml

# 验证服务存在
ros2 service list | grep dm_arm

# 服务调用
ros2 service call /dm_arm_server/eef_cmd dm_arm_msgs/srv/DmArmCmd \
  "{command: 'zero'}"

ros2 service call /dm_arm_server/eef_cmd dm_arm_msgs/srv/DmArmCmd \
  "{command: 'goal_base', x: 0.3, y: 0.0, z: 0.4, roll: 0.0, pitch: 1.57, yaw: 0.0}"

ros2 service call /dm_arm_server/eef_cmd dm_arm_msgs/srv/DmArmCmd \
  "{command: 'stretch', param1: '0.05'}"

ros2 service call /dm_arm_server/eef_cmd dm_arm_msgs/srv/DmArmCmd \
  "{command: 'get_pose'}"

# 多点任务
ros2 service call /dm_arm_server/task_planner dm_arm_msgs/srv/DmArmCmd \
  "{command: 'add_task', x: 0.3, y: 0.1, z: 0.4, param1: '0.5', param2: 'NONE'}"
ros2 service call /dm_arm_server/task_planner dm_arm_msgs/srv/DmArmCmd \
  "{command: 'add_task', x: 0.3, y: -0.1, z: 0.4, param1: '0.5', param2: 'STRETCH', param3: '0.03'}"
ros2 service call /dm_arm_server/task_planner dm_arm_msgs/srv/DmArmCmd \
  "{command: 'exe_all_tasks'}"
```

---

## 附录：完整差异速查表

| 内容 | ROS1 Noetic | ROS2 Humble |
|------|-------------|-------------|
| 节点初始化 | `ros::init(); ros::NodeHandle nh;` | `rclcpp::init(); auto node = rclcpp::Node::make_shared(...)` |
| MoveGroupInterface 构造 | `MoveGroupInterface arm("arm")` | `MoveGroupInterface arm(node, "arm")` |
| PlanningSceneMonitor 构造 | `PlanningSceneMonitor("robot_description")` | `PlanningSceneMonitor(node, "robot_description")` |
| TF2 Buffer 构造 | `tf2_ros::Buffer buf` | `tf2_ros::Buffer buf(node->get_clock())` |
| TF2 Listener 构造 | `TransformListener(buf)` | `TransformListener(buf, node)` |
| lookupTransform 时间 | `ros::Time(0)` | `tf2::TimePointZero` |
| 参数读取 | `nh.param<T>("key", val, default)` | `node->declare_parameter("key", default); node->get_parameter("key").as_T()` |
| 时间戳 | `ros::Time::now()` | `node->now()` |
| Duration sleep | `ros::Duration(1.0).sleep()` | `rclcpp::sleep_for(std::chrono::seconds(1))` |
| 消息命名空间 | `geometry_msgs::Pose` | `geometry_msgs::msg::Pose` |
| 服务类型 | `srv::Request / srv::Response` | `srv::Request::SharedPtr / Response::SharedPtr` |
| 服务端创建 | `nh.advertiseService("name", &Cls::cb, this)` | `create_service<T>("name", std::bind(...))` |
| 服务端回调签名 | `bool cb(Req& req, Res& res)` | `void cb(Req::SharedPtr req, Res::SharedPtr res)` |
| 服务客户端创建 | `nh.serviceClient<T>("name")` | `node->create_client<T>("name")` |
| 服务客户端调用 | `client.call(srv)` | `client->async_send_request(req)` + `spin_until_future_complete` |
| Spinner | `ros::AsyncSpinner spinner(n); spinner.start(); ros::waitForShutdown()` | `rclcpp::executors::MultiThreadedExecutor exec; exec.spin()` |
| 日志 | `ROS_INFO / ROS_WARN / ROS_ERROR` | `RCLCPP_INFO / RCLCPP_WARN / RCLCPP_ERROR` |
| 硬件接口基类 | `hardware_interface::RobotHW` | `hardware_interface::SystemInterface` |
| 硬件初始化 | `bool init(nh)` | `CallbackReturn on_init(HardwareInfo)` |
| 硬件接口注册 | `registerInterface(&joint_state_interface_)` | `export_state_interfaces()` 返回向量 |
| 硬件参数来源 | rosparam yaml | URDF `<ros2_control><hardware><param>` |
| 控制器配置 | `type: position_controllers/JointTrajectoryController` | `type: joint_trajectory_controller/JointTrajectoryController` |
| 插件注册 | `PLUGINLIB_EXPORT_CLASS(...)` | `PLUGINLIB_EXPORT_CLASS(...)` （相同） |
| plan() 返回值类型 | `moveit::planning_interface::MoveItErrorCode` | `moveit::core::MoveItErrorCode` |
| getCurrentState | `getCurrentState()` 无参数 | `getCurrentState(timeout_seconds)` 需指定超时 |
| PlanningScene 加锁 | `PlanningSceneMonitor::LockedPlanningSceneRO` | `LockedPlanningSceneRO` （相同） |
