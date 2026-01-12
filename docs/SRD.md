# Software Requirements Document (SRD) | 软件需求规格说明书
## RealMan Arm Driver for RecomoArm v1 | RecomoArm v1 睿尔曼机械臂驱动

**Version | 版本:** 1.0  
**Date | 日期:** 2026-01-12  
**Author | 作者:** RecomoArm Team

---

## 1. Introduction | 简介

### 1.1 Purpose | 目的
This document defines the software requirements for the RealMan Arm Driver, a ROS2 Humble package that enables control of a 3-DOF robotic arm via CAN FD communication on NVIDIA Jetson Orin.

本文档定义了睿尔曼机械臂驱动的软件需求，这是一个ROS2 Humble软件包，用于在NVIDIA Jetson Orin上通过CAN FD通信控制3自由度机械臂。

### 1.2 Scope | 范围
The driver provides:
- Low-level CAN FD communication with RealMan joint motors
- ROS2 interface for position, velocity, and current control modes
- Joint state feedback and motor status monitoring
- Safety features including error handling and limits enforcement

驱动程序提供：
- 与睿尔曼关节电机的底层CAN FD通信
- 用于位置、速度和电流控制模式的ROS2接口
- 关节状态反馈和电机状态监控
- 包括错误处理和限位保护的安全功能

### 1.3 Definitions | 术语定义

| Term 术语 | Definition 定义 |
|------|------------|
| CAN FD | Controller Area Network with Flexible Data-rate 灵活数据速率控制器局域网 |
| Joint 关节 | A single motor-driven rotational axis 单个电机驱动的旋转轴 |
| Servo Mode 伺服模式 | Closed-loop control mode (position/velocity/current) 闭环控制模式（位置/速度/电流） |
| IAP | In-Application Programming (firmware update) 在线应用编程（固件更新） |

---

## 2. System Overview | 系统概述

### 2.1 Hardware Configuration | 硬件配置

| Joint 关节 | Name 名称 | Motor Torque 电机扭矩 | CAN ID | Role 功能 |
|-------|------|--------------|--------|------|
| 1 | base_yaw | 10 Nm | 306 (0x132) | Base rotation 底座旋转 |
| 2 | base_pitch | 30 Nm | 305 (0x131) | Shoulder pitch 肩部俯仰 |
| 3 | elbow | 10 Nm | 304 (0x130) | Elbow pitch 肘部俯仰 |

### 2.2 Communication | 通信

- **Protocol 协议:** CAN FD (ISO 11898-1)
- **Arbitration Rate 仲裁速率:** 1 Mbps
- **Data Rate 数据速率:** 5 Mbps
- **Interface 接口:** SocketCAN (e.g., `can0`)

---

## 3. Functional Requirements | 功能需求

### 3.1 Control Modes | 控制模式

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| FR-01 | The driver SHALL support position servo mode (mode 3) 驱动程序应支持位置伺服模式（模式3） | Must 必须 |
| FR-02 | The driver SHALL support velocity servo mode (mode 2) 驱动程序应支持速度伺服模式（模式2） | Must 必须 |
| FR-03 | The driver SHALL support current/torque servo mode (mode 1) 驱动程序应支持电流/扭矩伺服模式（模式1） | Must 必须 |
| FR-04 | The driver SHALL allow runtime mode switching per joint 驱动程序应允许运行时切换各关节模式 | Should 应该 |

### 3.2 Motion Control | 运动控制

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| FR-10 | The driver SHALL accept joint position commands in radians 驱动程序应接受弧度单位的关节位置指令 | Must 必须 |
| FR-11 | The driver SHALL accept joint velocity commands in rad/s 驱动程序应接受rad/s单位的关节速度指令 | Must 必须 |
| FR-12 | The driver SHALL accept joint current commands in mA 驱动程序应接受mA单位的关节电流指令 | Must 必须 |
| FR-13 | The driver SHALL enforce configurable position limits 驱动程序应执行可配置的位置限制 | Must 必须 |
| FR-14 | The driver SHALL enforce configurable velocity limits 驱动程序应执行可配置的速度限制 | Should 应该 |
| FR-15 | The driver SHALL enforce configurable current limits 驱动程序应执行可配置的电流限制 | Should 应该 |

### 3.3 Feedback | 反馈

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| FR-20 | The driver SHALL publish joint positions at ≥50 Hz 驱动程序应以≥50 Hz发布关节位置 | Must 必须 |
| FR-21 | The driver SHALL publish joint velocities at ≥50 Hz 驱动程序应以≥50 Hz发布关节速度 | Must 必须 |
| FR-22 | The driver SHALL publish joint currents/efforts at ≥50 Hz 驱动程序应以≥50 Hz发布关节电流/力矩 | Must 必须 |
| FR-23 | The driver SHALL publish motor status (voltage, temp, errors) 驱动程序应发布电机状态（电压、温度、错误） | Must 必须 |

### 3.4 Safety & Error Handling | 安全与错误处理

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| FR-30 | The driver SHALL detect and report motor error codes 驱动程序应检测并报告电机错误代码 | Must 必须 |
| FR-31 | The driver SHALL provide error clearing capability 驱动程序应提供清除错误的功能 | Must 必须 |
| FR-32 | The driver SHALL support motor enable/disable 驱动程序应支持电机使能/禁用 | Must 必须 |
| FR-33 | The driver SHALL handle CAN communication timeouts 驱动程序应处理CAN通信超时 | Must 必须 |
| FR-34 | The driver SHALL log errors via ROS2 logging 驱动程序应通过ROS2日志记录错误 | Must 必须 |

### 3.5 Configuration | 配置

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| FR-40 | The driver SHALL load motor configuration from YAML 驱动程序应从YAML加载电机配置 | Must 必须 |
| FR-41 | The driver SHALL support zero position calibration 驱动程序应支持零位标定 | Should 应该 |
| FR-42 | The driver SHALL support configurable control loop rate 驱动程序应支持可配置的控制循环频率 | Must 必须 |

---

## 4. Non-Functional Requirements | 非功能需求

### 4.1 Performance | 性能

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| NFR-01 | Control loop latency SHALL be < 5 ms 控制循环延迟应小于5毫秒 | Must 必须 |
| NFR-02 | The driver SHALL support 100 Hz control rate 驱动程序应支持100 Hz控制频率 | Must 必须 |
| NFR-03 | CAN message round-trip SHALL be < 2 ms CAN消息往返时间应小于2毫秒 | Should 应该 |

### 4.2 Reliability | 可靠性

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| NFR-10 | The driver SHALL recover from transient CAN errors 驱动程序应能从瞬时CAN错误中恢复 | Must 必须 |
| NFR-11 | The driver SHALL maintain state consistency on reconnection 驱动程序应在重连时保持状态一致性 | Should 应该 |

### 4.3 Compatibility | 兼容性

| ID | Requirement 需求 | Priority 优先级 |
|----|-------------|----------|
| NFR-20 | The driver SHALL be compatible with ROS2 Humble 驱动程序应与ROS2 Humble兼容 | Must 必须 |
| NFR-21 | The driver SHALL run on Ubuntu 22.04 (Jetson Orin) 驱动程序应在Ubuntu 22.04（Jetson Orin）上运行 | Must 必须 |
| NFR-22 | The driver SHALL use standard ROS2 message types where possible 驱动程序应尽可能使用标准ROS2消息类型 | Must 必须 |

---

## 5. Interface Requirements | 接口需求

### 5.1 ROS2 Topics | ROS2话题

#### Publishers | 发布者

| Topic 话题 | Message Type 消息类型 | Rate 频率 | Description 描述 |
|-------|--------------|------|-------------|
| `/realman_arm/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | Position, velocity, effort 位置、速度、力矩 |
| `/realman_arm/status` | `realman_arm_driver/msg/ArmStatus` | 10 Hz | Voltage, temp, errors, enable state 电压、温度、错误、使能状态 |

#### Subscribers | 订阅者

| Topic 话题 | Message Type 消息类型 | Description 描述 |
|-------|--------------|-------------|
| `/realman_arm/joint_commands` | `sensor_msgs/msg/JointState` | Target position/velocity/effort 目标位置/速度/力矩 |

### 5.2 ROS2 Services | ROS2服务

| Service 服务 | Type 类型 | Description 描述 |
|---------|------|-------------|
| `/realman_arm/enable` | `std_srvs/srv/SetBool` | Enable (true) or disable (false) all motors 使能（true）或禁用（false）所有电机 |
| `/realman_arm/set_mode` | `realman_arm_driver/srv/SetMode` | Set control mode (1/2/3) 设置控制模式（1/2/3） |
| `/realman_arm/set_zero` | `std_srvs/srv/Trigger` | Set current position as zero 将当前位置设为零位 |
| `/realman_arm/clear_errors` | `std_srvs/srv/Trigger` | Clear all motor errors 清除所有电机错误 |

### 5.3 ROS2 Parameters | ROS2参数

| Parameter 参数 | Type 类型 | Default 默认值 | Description 描述 |
|-----------|------|---------|-------------|
| `can_interface` | string | "can0" | SocketCAN interface name SocketCAN接口名称 |
| `control_rate_hz` | double | 50.0 | Control loop frequency 控制循环频率 |
| `status_rate_hz` | double | 10.0 | Status publish frequency 状态发布频率 |
| `joint_names` | string[] | ["base_yaw", "base_pitch", "elbow"] | Joint names 关节名称 |
| `joint_ids` | int[] | [306, 305, 304] | CAN IDs CAN标识符 |
| `position_limits_min` | double[] | [-3.14, -1.57, -2.36] | Min position (rad) 最小位置（弧度） |
| `position_limits_max` | double[] | [3.14, 1.57, 2.36] | Max position (rad) 最大位置（弧度） |
| `velocity_limit` | double | 1.0 | Max velocity (rad/s) 最大速度（rad/s） |
| `current_limit` | double | 10000.0 | Max current (mA) 最大电流（mA） |

---

## 6. Constraints | 约束条件

- CAN FD hardware must support 5 Mbps data rate | CAN FD硬件必须支持5 Mbps数据速率
- SocketCAN kernel modules must be loaded | 必须加载SocketCAN内核模块
- Motors must have unique CAN IDs (no conflicts on bus) | 电机必须有唯一的CAN ID（总线上无冲突）
- IAP flag must be cleared before normal operation (address 0x49 = 0) | 正常运行前必须清除IAP标志（地址0x49 = 0）

---

## 7. Acceptance Criteria | 验收标准

| ID | Criterion 标准 |
|----|-----------|
| AC-01 | All three joints respond to position commands within 20ms 所有三个关节在20ms内响应位置指令 |
| AC-02 | Joint state feedback matches commanded position within 0.01 rad 关节状态反馈与指令位置误差在0.01弧度内 |
| AC-03 | Mode switching completes without errors 模式切换无错误完成 |
| AC-04 | Error conditions are detected and reported correctly 错误状态被正确检测和报告 |
| AC-05 | Driver recovers gracefully from CAN bus disconnection 驱动程序能从CAN总线断开中优雅恢复 |
