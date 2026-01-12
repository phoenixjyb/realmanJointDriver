# RealMan Arm Driver | 睿尔曼机械臂驱动

ROS2 Humble driver for RealMan joint motors via CAN FD, designed for the RecomoArm 3-DOF robotic arm.

基于ROS2 Humble的睿尔曼关节电机CAN FD驱动程序，专为RecomoArm 3自由度机械臂设计。

## Overview | 概述

This package provides a ROS2 node that communicates with RealMan joint motors over CAN FD (SocketCAN). It supports:

本软件包提供一个通过CAN FD（SocketCAN）与睿尔曼关节电机通信的ROS2节点。支持：

- **Position control mode | 位置控制模式** - servo to target joint positions 伺服到目标关节位置
- **Velocity control mode | 速度控制模式** - servo to target joint velocities 伺服到目标关节速度
- **Current/Torque control mode | 电流/扭矩控制模式** - direct current control for force applications 用于力控应用的直接电流控制

## Hardware Configuration | 硬件配置

| Joint 关节 | Name 名称 | Torque 扭矩 | CAN ID | Role 功能 |
|-------|------|--------|--------|------|
| 1 | base_yaw | 10 Nm | 11 | Base rotation 底座旋转 |
| 2 | base_pitch | 30 Nm | 12 | Shoulder pitch 肩部俯仰 |
| 3 | elbow | 10 Nm | 13 | Elbow pitch 肘部俯仰 |

> **Note:** Valid CAN IDs are 1-30. Factory default is ID=1 for all motors.
> 
> **注意：** 有效的CAN ID为1-30。出厂默认所有电机ID都是1。

## Prerequisites | 前置条件

### System Requirements | 系统要求
- Ubuntu 22.04 (Jetson Orin)
- ROS2 Humble
- SocketCAN kernel modules | SocketCAN内核模块

### CAN Interface Setup | CAN接口设置

1. Load CAN kernel modules | 加载CAN内核模块:
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_fd
```

2. Configure CAN interface (adjust for your hardware) | 配置CAN接口（根据硬件调整）:
```bash
# For USB-CAN adapter | USB-CAN适配器
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

# Verify | 验证
ip -details link show can0
```

3. To make persistent, add to `/etc/network/interfaces.d/can0` | 持久化配置:
```
auto can0
iface can0 can static
    bitrate 1000000
    dbitrate 5000000
    fd on
```

## Motor Setup Utility | 电机设置工具

Before using the driver, you need to configure unique CAN IDs for each motor.

使用驱动前，需要为每个电机配置唯一的CAN ID。

### Scan for motors | 扫描电机

```bash
cd src/realman_arm_driver/scripts
python3 motor_setup.py -i can0 --scan
```

### Change motor ID | 更改电机ID

**Important:** Connect only ONE motor at a time when changing IDs!

**重要：** 更改ID时每次只连接一个电机！

```bash
# Setup motor 1 (base_yaw): ID 1 -> 11
# 设置电机1（底座偏航）：ID 1 -> 11
python3 motor_setup.py -i can0 --current-id 1 --new-id 11
# Power cycle the motor! 给电机断电重启！

# Setup motor 2 (base_pitch): ID 1 -> 12
# 设置电机2（底座俯仰）：ID 1 -> 12
python3 motor_setup.py -i can0 --current-id 1 --new-id 12
# Power cycle the motor! 给电机断电重启！

# Setup motor 3 (elbow): ID 1 -> 13
# 设置电机3（肘部）：ID 1 -> 13
python3 motor_setup.py -i can0 --current-id 1 --new-id 13
# Power cycle the motor! 给电机断电重启！
```

### Other commands | 其他命令

```bash
# Get motor status | 获取电机状态
python3 motor_setup.py -i can0 --id 11 --status

# Clear errors | 清除错误
python3 motor_setup.py -i can0 --id 11 --clear-errors

# Set zero position | 设置零位
python3 motor_setup.py -i can0 --id 11 --set-zero

# Enable/disable motor | 使能/禁用电机
python3 motor_setup.py -i can0 --id 11 --enable
python3 motor_setup.py -i can0 --id 11 --disable

# Clear IAP flag (required before operation) | 清除IAP标志
python3 motor_setup.py -i can0 --id 11 --clear-iap
```

## Installation | 安装

### Build from source | 从源码构建

```bash
cd ~/ros2_ws/src
ln -s /path/to/recomoArm-v1/src/realman_arm_driver .

cd ~/ros2_ws
colcon build --packages-select realman_arm_driver
source install/setup.bash
```

## Usage | 使用方法

### Launch the driver | 启动驱动

```bash
ros2 launch realman_arm_driver arm_driver.launch.py
```

With custom CAN interface | 使用自定义CAN接口:
```bash
ros2 launch realman_arm_driver arm_driver.launch.py can_interface:=can1
```

### ROS2 Interface | ROS2接口

#### Published Topics | 发布话题

| Topic 话题 | Type 类型 | Rate 频率 | Description 描述 |
|-------|------|------|-------------|
| `~/joint_states` | `sensor_msgs/JointState` | 50 Hz | Joint positions, velocities, efforts 关节位置、速度、力矩 |
| `~/status` | `realman_arm_driver/ArmStatus` | 10 Hz | Motor status (voltage, temp, errors) 电机状态（电压、温度、错误） |

#### Subscribed Topics | 订阅话题

| Topic 话题 | Type 类型 | Description 描述 |
|-------|------|-------------|
| `~/joint_commands` | `sensor_msgs/JointState` | Target positions/velocities/efforts 目标位置/速度/力矩 |

#### Services | 服务

| Service 服务 | Type 类型 | Description 描述 |
|---------|------|-------------|
| `~/enable` | `std_srvs/SetBool` | Enable (true) or disable (false) motors 使能或禁用电机 |
| `~/set_mode` | `realman_arm_driver/SetMode` | Set control mode (1=current, 2=velocity, 3=position) 设置控制模式 |
| `~/set_zero` | `std_srvs/Trigger` | Set current position as zero 将当前位置设为零位 |
| `~/clear_errors` | `std_srvs/Trigger` | Clear motor error codes 清除电机错误代码 |

### Example Commands | 示例命令

```bash
# Enable motors | 使能电机
ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: true}"

# Set position mode | 设置位置模式
ros2 service call /realman_arm_driver/set_mode realman_arm_driver/srv/SetMode "{mode: 3}"

# Send position command (radians) | 发送位置指令（弧度）
ros2 topic pub /realman_arm_driver/joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_yaw', 'base_pitch', 'elbow'], position: [0.0, 0.5, -0.3]}"

# Monitor joint states | 监控关节状态
ros2 topic echo /realman_arm_driver/joint_states

# Check status | 检查状态
ros2 topic echo /realman_arm_driver/status

# Clear errors | 清除错误
ros2 service call /realman_arm_driver/clear_errors std_srvs/srv/Trigger

# Disable motors | 禁用电机
ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: false}"
```

## Configuration | 配置

Edit `config/arm_config.yaml` to customize | 编辑配置文件自定义参数:

```yaml
realman_arm_driver:
  ros__parameters:
    can_interface: "can0"          # CAN接口名称
    control_rate_hz: 50.0          # 控制循环频率
    status_rate_hz: 10.0           # 状态发布频率
    
    joint_names: ["base_yaw", "base_pitch", "elbow"]
    joint_ids: [11, 12, 13]                       # 关节CAN ID (valid: 1-30)
    
    # Position limits (radians) | 位置限制（弧度）
    position_limits_min: [-3.14159, -1.5708, -2.3562]
    position_limits_max: [3.14159, 1.5708, 2.3562]
    
    # Velocity limits (rad/s) | 速度限制
    velocity_limits: [1.0, 1.0, 1.0]
    
    # Current limits (mA) | 电流限制
    current_limits: [10000.0, 20000.0, 10000.0]
    
    default_mode: 3                # 默认模式（位置模式）
    enable_on_startup: false       # 启动时是否使能
```

## Error Codes | 错误代码

| Code 代码 | Meaning 含义 |
|------|---------|
| 0x0001 | FOC rate too high FOC频率过高 |
| 0x0002 | Over voltage 过压 |
| 0x0004 | Under voltage 欠压 |
| 0x0008 | Over temperature 过温 |
| 0x0010 | Startup failed 启动失败 |
| 0x0020 | Encoder error 编码器错误 |
| 0x0040 | Over current 过流 |
| 0x0080 | Software error 软件错误 |
| 0x0100 | Temperature sensor error 温度传感器错误 |
| 0x0200 | Position limit exceeded 位置超限 |
| 0x0400 | Invalid joint ID 关节ID非法 |
| 0x0800 | Position tracking error 位置跟踪误差超限 |
| 0x1000 | Current sensor error 电流检测错误 |
| 0x2000 | Brake failure 抱闸失败 |
| 0x4000 | Position command step 位置指令阶跃 |
| 0x8000 | Multi-turn data lost 多圈丢数 |

## Troubleshooting | 故障排除

### CAN interface not found | 找不到CAN接口
```bash
# Check if interface exists | 检查接口是否存在
ip link show can0

# If not, check kernel modules | 检查内核模块
lsmod | grep can
```

### No response from motors | 电机无响应
1. Verify CAN wiring and termination | 检查CAN接线和终端电阻
2. Check motor power supply | 检查电机电源
3. Verify CAN IDs match configuration | 确认CAN ID与配置匹配
4. Try lower baud rate for debugging | 尝试降低波特率进行调试

### Motors won't enable | 电机无法使能
1. Check for error codes in `~/status` topic | 检查状态话题中的错误代码
2. Call `~/clear_errors` service | 调用清除错误服务
3. Verify motors are in correct mode | 确认电机处于正确模式

## License | 许可证

MIT License

## References | 参考文档

- [CAN Protocol Documentation | CAN协议文档](../refs/jointMotorCAN_v1.0.md)
- [Software Requirements Document | 软件需求规格说明书](../docs/SRD.md)
- [Software Design Document | 软件设计文档](../docs/SDD.md)
