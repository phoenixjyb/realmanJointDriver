# RecomoArm v1 | 睿科机械臂 v1

A 3-DOF robotic arm project based on RealMan joint motors, controlled via ROS2 on NVIDIA Jetson Orin.

基于睿尔曼关节电机的3自由度机械臂项目，通过NVIDIA Jetson Orin上的ROS2进行控制。

## Project Structure | 项目结构

```
recomoArm-v1/
├── docs/                           # Documentation | 文档
│   ├── SRD.md                      # Software Requirements Document | 软件需求规格说明书
│   └── SDD.md                      # Software Design Document | 软件设计文档
├── refs/                           # Reference materials | 参考资料
│   └── jointMotorCAN_v1.0.md       # CAN protocol documentation | CAN协议文档
└── src/                            # Source code | 源代码
    └── realman_arm_driver/         # ROS2 driver package | ROS2驱动包
```

## Hardware | 硬件

| Component 组件 | Specification 规格 |
|------------|--------------|
| Controller 控制器 | NVIDIA Jetson Orin |
| Joint 1 关节1 | RealMan 10Nm motor (base_yaw 底座偏航) |
| Joint 2 关节2 | RealMan 30Nm motor (base_pitch 底座俯仰) |
| Joint 3 关节3 | RealMan 10Nm motor (elbow 肘部) |
| Communication 通信 | CAN FD (1Mbps/5Mbps) |

## Quick Start | 快速开始

### On Jetson Orin | 在Jetson Orin上

```bash
# Setup CAN interface | 设置CAN接口
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up

# Build | 构建
cd /home/nvidia/yanbo/armv1_ws
source /opt/ros/humble/setup.bash
colcon build

# Run | 运行
source install/setup.bash
ros2 launch realman_arm_driver arm_driver.launch.py
```

### Control the arm | 控制机械臂

```bash
# Enable motors | 使能电机
ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: true}"

# Send position command (degrees) | 发送位置指令（度）
ros2 topic pub /realman_arm_driver/joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_yaw', 'base_pitch', 'elbow'], position: [0.0, 30.0, -20.0]}"

Note: position commands are in degrees, and the driver limits each command step to 20° (larger moves are split automatically). | 注意：位置指令单位为度，驱动会将单次指令步长限制为 20°（更大的角度会自动拆分）。
```

## Documentation | 文档

- **[Quick Start Guide | 快速启动指南](docs/QUICKSTART.md)** - Complete step-by-step bringup 完整分步启动
- **[Handover Debug | 调试问题交接](docs/HANDOVER_DEBUG.md)** - IAP flag clearing fix (2026-01-16) IAP标志清除修复
- [Driver README | 驱动说明](src/realman_arm_driver/README.md)
- [Software Requirements | 软件需求](docs/SRD.md)
- [Software Design | 软件设计](docs/SDD.md)
- [CAN Protocol | CAN协议](refs/jointMotorCAN_v1.0.md)

## Recent Updates | 最近更新

### 2026-01-17: Control & Status Updates | 控制与状态更新

- Position commands use degrees; driver converts to radians internally, and status now includes `position_deg`.
- Position commands are step-limited to 20° per control cycle (larger moves are split).
- Auto-enable triggers after communication is ready (default `enable_on_startup: true` in `config/arm_config.yaml`).

- 位置指令单位改为度；驱动内部转换为弧度，状态中新增 `position_deg`。
- 位置指令每个控制周期限制为 20°（更大角度自动拆分）。
- 通信就绪后自动使能（`config/arm_config.yaml` 默认 `enable_on_startup: true`）。

### 2026-01-16: IAP Flag Clearing Fix | IAP标志清除修复

**Problem 问题**: Motors not responding on startup, continuous CAN errors  
电机启动时无响应，持续CAN错误

**Solution 解决方案**: Enhanced IAP flag clearing with proper verification  
增强IAP标志清除，添加正确的验证机制

**Quick Test 快速测试**:
```bash
./test_iap_clear.sh
```

**Details 详情**: See [HANDOVER_DEBUG.md](docs/HANDOVER_DEBUG.md) and [IAP_FIX_QUICKREF.txt](IAP_FIX_QUICKREF.txt)

## License | 许可证

MIT License
