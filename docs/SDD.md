# Software Design Document (SDD) | 软件设计文档
## RealMan Arm Driver for RecomoArm v1 | RecomoArm v1 睿尔曼机械臂驱动

**Version | 版本:** 1.0  
**Date | 日期:** 2026-01-12  
**Author | 作者:** RecomoArm Team

---

## 1. Architecture Overview | 架构概述

### 1.1 System Context | 系统上下文

```
┌─────────────────────────────────────────────────────────────────┐
│                        Jetson Orin                               │
│  ┌──────────────┐     ┌─────────────────────────────────────┐   │
│  │  IK Solver   │     │      realman_arm_driver node        │   │
│  │  逆运动学求解 │────▶│      睿尔曼机械臂驱动节点            │   │
│  │  (10 Hz)     │     │                                     │   │
│  └──────────────┘     │  ┌─────────┐  ┌──────────────────┐  │   │
│                       │  │ ROS2    │  │  CAN Protocol    │  │   │
│  ┌──────────────┐     │  │ Layer   │  │  Layer           │  │   │
│  │  Telemetry   │◀────│  │ ROS2层  │  │  CAN协议层        │  │   │
│  │  Monitor     │     │  └─────────┘  └────────┬─────────┘  │   │
│  │  遥测监控     │     │                        │             │   │
│  └──────────────┘     └────────────────────────┼─────────────┘   │
│                                                │                  │
│                                    ┌───────────▼───────────┐     │
│                                    │    SocketCAN (can0)   │     │
│                                    └───────────┬───────────┘     │
└────────────────────────────────────────────────┼─────────────────┘
                                                 │ CAN FD Bus | CAN FD总线
                    ┌────────────────────────────┼────────────────────────────┐
                    │                            │                            │
             ┌──────▼──────┐             ┌───────▼─────┐             ┌────────▼────┐
             │  Joint 1    │             │  Joint 2    │             │  Joint 3    │
             │  关节1       │             │  关节2       │             │  关节3       │
             │  base_yaw   │             │  base_pitch │             │  elbow      │
             │  底座偏航    │             │  底座俯仰    │             │  肘部        │
             │  ID: 306    │             │  ID: 305    │             │  ID: 304    │
             │  10 Nm      │             │  30 Nm      │             │  10 Nm      │
             └─────────────┘             └─────────────┘             └─────────────┘
```

### 1.2 Package Structure | 包结构

```
src/realman_arm_driver/
├── CMakeLists.txt
├── package.xml
├── include/realman_arm_driver/
│   ├── can_protocol.hpp        # CAN FD protocol | CAN FD协议定义与实现
│   ├── realman_arm_node.hpp    # ROS2 node class | ROS2节点类定义
│   └── types.hpp               # Common types | 通用类型和常量
├── src/
│   ├── can_protocol.cpp        # CAN protocol impl | CAN协议实现
│   ├── realman_arm_node.cpp    # ROS2 node impl | ROS2节点实现
│   └── main.cpp                # Entry point | 入口点
├── msg/
│   ├── ArmStatus.msg           # Arm status message | 机械臂状态消息
│   └── JointStatus.msg         # Joint status message | 关节状态消息
├── srv/
│   └── SetMode.srv             # Mode setting service | 模式设置服务
├── config/
│   └── arm_config.yaml         # Default config | 默认配置
└── launch/
    └── arm_driver.launch.py    # Launch file | 启动文件
```

---

## 2. Component Design | 组件设计

### 2.1 Class Diagram | 类图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        RealmanArmNode                                │
│                        睿尔曼机械臂节点                                │
│─────────────────────────────────────────────────────────────────────│
│ - can_interface_: CanInterface          // CAN接口                   │
│ - joints_: vector<JointConfig>          // 关节配置                   │
│ - joint_states_: JointState             // 关节状态                   │
│ - control_mode_: ControlMode            // 控制模式                   │
│ - enabled_: bool                        // 使能状态                   │
│─────────────────────────────────────────────────────────────────────│
│ + RealmanArmNode()                      // 构造函数                   │
│ - controlLoopCallback()                 // 控制循环回调                │
│ - statusLoopCallback()                  // 状态循环回调                │
│ - jointCommandCallback(msg)             // 关节指令回调                │
│ - enableService(req, res)               // 使能服务                   │
│ - setModeService(req, res)              // 设置模式服务               │
│ - setZeroService(req, res)              // 设置零位服务               │
│ - clearErrorsService(req, res)          // 清除错误服务               │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ uses | 使用
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         CanInterface                                 │
│                         CAN接口类                                     │
│─────────────────────────────────────────────────────────────────────│
│ - socket_fd_: int                       // 套接字文件描述符           │
│ - interface_name_: string               // 接口名称                   │
│─────────────────────────────────────────────────────────────────────│
│ + CanInterface(interface_name)          // 构造函数                   │
│ + ~CanInterface()                       // 析构函数                   │
│ + open(): bool                          // 打开接口                   │
│ + close(): void                         // 关闭接口                   │
│ + sendFrame(id, data, len): bool        // 发送帧                     │
│ + receiveFrame(timeout_ms): optional    // 接收帧                     │
│ + sendPositionCommand(id, pos): Feedback // 发送位置指令              │
│ + sendVelocityCommand(id, vel): Feedback // 发送速度指令              │
│ + sendCurrentCommand(id, cur): Feedback  // 发送电流指令              │
│ + queryStatus(id): JointStatus          // 查询状态                   │
│ + writeRegister(id, addr, val): bool    // 写寄存器                   │
│ + readRegister(id, addr, cnt): vector   // 读寄存器                   │
│ + setEnabled(id, enabled): bool         // 设置使能                   │
│ + setMode(id, mode): bool               // 设置模式                   │
│ + setZero(id): bool                     // 设置零位                   │
│ + clearErrors(id): bool                 // 清除错误                   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Types | 数据类型

```cpp
// Control modes | 控制模式 (matches motor protocol | 与电机协议一致)
enum class ControlMode : uint8_t {
    OPEN_LOOP = 0,   // 开环模式
    CURRENT = 1,     // 电流模式
    VELOCITY = 2,    // 速度模式
    POSITION = 3     // 位置模式
};

// Joint configuration | 关节配置
struct JointConfig {
    std::string name;     // 关节名称
    uint16_t can_id;      // CAN标识符
    double position_min;  // 最小位置 (radians | 弧度)
    double position_max;  // 最大位置 (radians | 弧度)
    double velocity_max;  // 最大速度 (rad/s)
    double current_max;   // 最大电流 (mA)
    double gear_ratio;    // 减速比
};

// Servo feedback from motor | 电机伺服反馈
struct ServoFeedback {
    int32_t current_ma;      // 电流 (mA)
    int32_t velocity_raw;    // 速度 (0.02 RPM units | 0.02 RPM单位)
    int32_t position_raw;    // 位置 (0.0001 degree units | 0.0001度单位)
    uint8_t enabled;         // 使能状态
    uint16_t error_code;     // 错误代码
};

// Status query feedback | 状态查询反馈
struct JointStatus {
    uint16_t error_code;     // 错误代码
    uint16_t voltage_raw;    // 电压 (0.01V units | 0.01V单位)
    uint16_t temp_raw;       // 温度 (0.1°C units | 0.1°C单位)
    uint8_t enabled;         // 使能状态
    uint8_t brake_state;     // 抱闸状态
    int32_t position_raw;    // 位置
    int32_t current_raw;     // 电流
};
```

---

## 3. CAN Protocol Implementation | CAN协议实现

### 3.1 CAN ID Mapping | CAN ID映射

| Operation 操作 | TX CAN ID 发送ID | RX CAN ID 接收ID | Data Length 数据长度 |
|-----------|-----------|-----------|-------------|
| Read/Write Register 读写寄存器 | `joint_id` | `joint_id + 0x100` | Variable 可变 |
| Position Servo 位置伺服 | `joint_id + 0x200` | `joint_id + 0x500` | 4 / 16 |
| Velocity Servo 速度伺服 | `joint_id + 0x300` | `joint_id + 0x500` | 4 / 16 |
| Current Servo 电流伺服 | `joint_id + 0x400` | `joint_id + 0x500` | 4 / 16 |
| Status Query 状态查询 | `joint_id + 0x600` | `joint_id + 0x700` | 0 / 16 |

### 3.2 Unit Conversions | 单位转换

```cpp
// Position: radians <-> 0.0001 degree units
// 位置：弧度 <-> 0.0001度单位
inline int32_t radiansToRaw(double radians) {
    return static_cast<int32_t>(radians * 180.0 / M_PI * 10000.0);
}

inline double rawToRadians(int32_t raw) {
    return static_cast<double>(raw) / 10000.0 * M_PI / 180.0;
}

// Velocity: rad/s <-> 0.002 RPM units (for command)
// 速度：rad/s <-> 0.002 RPM单位（用于指令）
inline int32_t radPerSecToRawCmd(double rad_per_sec) {
    double rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
    return static_cast<int32_t>(rpm / 0.002);
}

// Velocity: rad/s <-> 0.02 RPM units (for feedback)
// 速度：rad/s <-> 0.02 RPM单位（用于反馈）
inline double rawFeedbackToRadPerSec(int32_t raw) {
    double rpm = static_cast<double>(raw) * 0.02;
    return rpm * 2.0 * M_PI / 60.0;
}

// Current: mA (direct for 10Nm/30Nm joints)
// 电流：mA（10Nm/30Nm关节直接使用）
inline int32_t currentToRaw(double current_ma) {
    return static_cast<int32_t>(current_ma);
}
```

### 3.3 Frame Format | 帧格式

**Position Command Frame | 位置指令帧:**
```
Byte 字节:  [0]    [1]    [2]    [3]
Data 数据:  pos_L  pos_ML pos_MH pos_H   (little-endian int32 | 小端序32位整数)
```

**Servo Feedback Frame (16 bytes) | 伺服反馈帧（16字节）:**
```
Byte 字节:  [0-3]      [4-7]      [8-11]     [12-13]  [14-15]
Data 数据:  current    velocity   position   enabled  error_code
            电流       速度       位置       使能状态  错误代码
            (int32)    (int32)    (int32)    (uint16) (uint16)
```

---

## 4. Control Loop Design | 控制循环设计

### 4.1 Timing | 时序

```
┌─────────────────────────────────────────────────────────────────────┐
│              Control Loop (50 Hz / 20ms) | 控制循环（50 Hz / 20ms）   │
│─────────────────────────────────────────────────────────────────────│
│  T=0ms    T=2ms    T=4ms    T=6ms              T=18ms    T=20ms     │
│    │        │        │        │                   │         │        │
│    ▼        ▼        ▼        ▼                   ▼         ▼        │
│  [CMD1]  [CMD2]  [CMD3]  [Publish]           [Wait]    [Next]       │
│  Joint1  Joint2  Joint3  JointState                    Cycle        │
│  关节1   关节2   关节3   发布关节状态           等待     下一周期     │
│  TX/RX   TX/RX   TX/RX                                              │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 Control Loop Pseudocode | 控制循环伪代码

```cpp
void RealmanArmNode::controlLoopCallback() {
    // 1. Send commands and receive feedback for each joint
    // 1. 向每个关节发送指令并接收反馈
    for (size_t i = 0; i < joints_.size(); ++i) {
        ServoFeedback fb;
        
        switch (control_mode_) {
            case ControlMode::POSITION:  // 位置模式
                fb = can_interface_.sendPositionCommand(
                    joints_[i].can_id, 
                    target_positions_[i]
                );
                break;
            case ControlMode::VELOCITY:  // 速度模式
                fb = can_interface_.sendVelocityCommand(
                    joints_[i].can_id,
                    target_velocities_[i]
                );
                break;
            case ControlMode::CURRENT:   // 电流模式
                fb = can_interface_.sendCurrentCommand(
                    joints_[i].can_id,
                    target_currents_[i]
                );
                break;
        }
        
        // Update joint states from feedback | 从反馈更新关节状态
        joint_states_.position[i] = rawToRadians(fb.position_raw);
        joint_states_.velocity[i] = rawFeedbackToRadPerSec(fb.velocity_raw);
        joint_states_.effort[i] = fb.current_ma / 1000.0;  // Convert to A | 转换为安培
        
        // Check for errors | 检查错误
        if (fb.error_code != 0) {
            RCLCPP_WARN(get_logger(), "Joint %s error: 0x%04X", 
                        joints_[i].name.c_str(), fb.error_code);
        }
    }
    
    // 2. Publish joint states | 2. 发布关节状态
    joint_states_.header.stamp = now();
    joint_state_pub_->publish(joint_states_);
}
```

---

## 5. Error Handling | 错误处理

### 5.1 Error Code Definitions (from protocol) | 错误代码定义（来自协议）

| Bit 位 | Code 代码 | Description 描述 | Recovery 恢复方法 |
|-----|------|-------------|----------|
| 0 | 0x0001 | FOC rate too high FOC频率过高 | Clear errors 清除错误 |
| 1 | 0x0002 | Over voltage 过压 | Check power supply 检查电源 |
| 2 | 0x0004 | Under voltage 欠压 | Check power supply 检查电源 |
| 3 | 0x0008 | Over temperature 过温 | Wait to cool 等待冷却 |
| 4 | 0x0010 | Startup failed 启动失败 | Reset motor 重启电机 |
| 5 | 0x0020 | Encoder error 编码器错误 | Check encoder 检查编码器 |
| 6 | 0x0040 | Over current 过流 | Reduce load 减轻负载 |
| 7 | 0x0080 | Software error 软件错误 | Reset motor 重启电机 |
| 8 | 0x0100 | Temperature sensor error 温度传感器错误 | Check sensor 检查传感器 |
| 9 | 0x0200 | Position limit exceeded 位置超限 | Move within limits 移动到限位内 |
| 10 | 0x0400 | Invalid joint ID 关节ID非法 | Check configuration 检查配置 |
| 11 | 0x0800 | Position tracking error 位置跟踪误差超限 | Reduce speed 降低速度 |
| 12 | 0x1000 | Current sensor error 电流检测错误 | Check sensor 检查传感器 |
| 13 | 0x2000 | Brake failure 抱闸失败 | Check brake 检查抱闸 |
| 14 | 0x4000 | Position command step 位置指令阶跃 | Smooth command 平滑指令 |
| 15 | 0x8000 | Multi-turn data lost 多圈丢数 | Re-calibrate 重新标定 |

### 5.2 Communication Error Handling | 通信错误处理

```cpp
// Timeout handling | 超时处理
if (!can_interface_.receiveFrame(TIMEOUT_MS)) {
    consecutive_errors_++;
    if (consecutive_errors_ > MAX_CONSECUTIVE_ERRORS) {
        RCLCPP_ERROR(get_logger(), "CAN communication lost, disabling motors");
        // CAN通信丢失，禁用电机
        emergencyStop();
    }
} else {
    consecutive_errors_ = 0;
}
```

---

## 6. Configuration | 配置

### 6.1 YAML Configuration Schema | YAML配置模式

```yaml
realman_arm_driver:
  ros__parameters:
    # CAN interface | CAN接口
    can_interface: "can0"
    
    # Control rates | 控制频率
    control_rate_hz: 50.0    # 控制循环频率
    status_rate_hz: 10.0     # 状态发布频率
    
    # Communication | 通信设置
    can_timeout_ms: 10       # CAN响应超时
    max_consecutive_errors: 5 # 最大连续错误次数
    
    # Joints | 关节配置
    joint_names: ["base_yaw", "base_pitch", "elbow"]
    joint_ids: [306, 305, 304]
    
    # Position limits (radians) | 位置限制（弧度）- 待配置
    position_limits_min: [-3.14159, -1.5708, -2.3562]
    position_limits_max: [3.14159, 1.5708, 2.3562]
    
    # Velocity limits (rad/s) | 速度限制
    velocity_limits: [1.0, 1.0, 1.0]
    
    # Current limits (mA) | 电流限制
    current_limits: [10000, 20000, 10000]
    
    # Initial mode | 初始模式
    default_mode: 3  # Position mode | 位置模式
    
    # Enable on startup | 启动时使能
    enable_on_startup: false
```

---

## 7. Launch Configuration | 启动配置

```python
# arm_driver.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('realman_arm_driver'),
        'config',
        'arm_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='realman_arm_driver',
            executable='realman_arm_node',
            name='realman_arm_driver',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
```

---

## 8. Testing Strategy | 测试策略

### 8.1 Unit Tests | 单元测试
- CAN frame encoding/decoding | CAN帧编解码
- Unit conversion functions | 单位转换函数
- Configuration loading | 配置加载

### 8.2 Integration Tests | 集成测试
- Motor communication (loopback or with hardware) | 电机通信（回环或硬件）
- ROS2 topic/service verification | ROS2话题/服务验证
- Mode switching | 模式切换

### 8.3 System Tests | 系统测试
- Full arm motion with IK upstream | 配合上游逆运动学的完整机械臂运动
- Error injection and recovery | 错误注入与恢复
- Long-duration stability | 长时间稳定性

---

## 10. Motor Setup Process | 电机设置流程

Before using the driver, each motor must be configured with a unique CAN ID.

使用驱动前，必须为每个电机配置唯一的CAN ID。

### 10.1 Factory Default | 出厂默认

All motors ship with **CAN ID = 1**. They must be configured individually.

所有电机出厂时**CAN ID = 1**。必须单独配置每个电机。

### 10.2 Target Configuration | 目标配置

| Joint 关节 | Name 名称 | Target ID 目标ID |
|-------|------|------------|
| 1 | base_yaw 底座偏航 | 11 |
| 2 | base_pitch 底座俯仰 | 12 |
| 3 | elbow 肘部 | 13 |

### 10.3 Setup Procedure | 设置步骤

```
┌─────────────────────────────────────────────────────────────────────┐
│                Motor ID Setup Process | 电机ID设置流程               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  Prerequisites | 前提条件:                                           │
│  - CAN interface up | CAN接口已启动                                  │
│  - Motors powered | 电机已上电                                       │
│                                                                      │
│  Step 1: Connect ONLY motor 1 to CAN bus                            │
│  步骤1：只将电机1连接到CAN总线                                        │
│                                                                      │
│  Step 2: Change ID from 1 to 11                                     │
│  步骤2：将ID从1改为11                                                 │
│    $ python3 motor_setup.py -i can0 --current-id 1 --new-id 11      │
│                                                                      │
│  Step 3: Power cycle motor 1                                        │
│  步骤3：给电机1断电重启                                               │
│                                                                      │
│  Step 4: Disconnect motor 1, connect ONLY motor 2                   │
│  步骤4：断开电机1，只连接电机2                                        │
│                                                                      │
│  Step 5: Change ID from 1 to 12                                     │
│  步骤5：将ID从1改为12                                                 │
│    $ python3 motor_setup.py -i can0 --current-id 1 --new-id 12      │
│                                                                      │
│  Step 6: Power cycle motor 2                                        │
│  步骤6：给电机2断电重启                                               │
│                                                                      │
│  Step 7: Disconnect motor 2, connect ONLY motor 3                   │
│  步骤7：断开电机2，只连接电机3                                        │
│                                                                      │
│  Step 8: Change ID from 1 to 13                                     │
│  步骤8：将ID从1改为13                                                 │
│    $ python3 motor_setup.py -i can0 --current-id 1 --new-id 13      │
│                                                                      │
│  Step 9: Power cycle motor 3                                        │
│  步骤9：给电机3断电重启                                               │
│                                                                      │
│  Step 10: Connect all motors and verify                             │
│  步骤10：连接所有电机并验证                                           │
│    $ python3 motor_setup.py -i can0 --scan                          │
│    Expected: Found motors at ID 11, 12, 13                          │
│    预期结果：在ID 11, 12, 13发现电机                                  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 10.4 ID Change Protocol | ID更改协议

The ID change process writes to the motor's memory control table:

ID更改过程向电机的内存控制表写入数据：

```cpp
// 1. Disable motor | 禁用电机
writeRegister(current_id, 0x0A, 0);  // SYS_ENABLE_DRIVER = 0

// 2. Write new ID | 写入新ID
writeRegister(current_id, 0x01, new_id);  // SYS_ID = new_id

// 3. Save to Flash | 保存到Flash
writeRegister(current_id, 0x0C, 1);  // SYS_SAVE_TO_FLASH = 1

// 4. Power cycle required! | 需要断电重启！
```

### 10.5 Motor Setup Utility | 电机设置工具

Location | 位置: `scripts/motor_setup.py`

Commands | 命令:

| Command 命令 | Description 描述 |
|---------|-------------|
| `--scan` | Scan for motors on bus 扫描总线上的电机 |
| `--id N --status` | Get motor N status 获取电机N状态 |
| `--current-id N --new-id M` | Change ID from N to M 将ID从N改为M |
| `--id N --clear-errors` | Clear errors on motor N 清除电机N的错误 |
| `--id N --set-zero` | Set current position as zero 将当前位置设为零位 |
| `--id N --enable` | Enable motor N 使能电机N |
| `--id N --disable` | Disable motor N 禁用电机N |
| `--id N --clear-iap` | Clear IAP flag 清除IAP标志 |

---

## 11. Future Considerations | 未来考虑

1. **ros2_control Integration | ros2_control集成**: Implement hardware interface for MoveIt2 compatibility 实现硬件接口以兼容MoveIt2
2. **Trajectory Execution | 轨迹执行**: Add JointTrajectory action server 添加关节轨迹动作服务器
3. **Diagnostics | 诊断**: Integrate with ROS2 diagnostics framework 集成ROS2诊断框架
4. **Real-time | 实时性**: Consider real-time Linux kernel for tighter timing 考虑实时Linux内核以获得更严格的时序控制
