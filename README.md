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

# Send position command | 发送位置指令
ros2 topic pub /realman_arm_driver/joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_yaw', 'base_pitch', 'elbow'], position: [0.0, 0.5, -0.3]}"
```

## Documentation | 文档

- **[Quick Start Guide | 快速启动指南](docs/QUICKSTART.md)** - Complete step-by-step bringup 完整分步启动
- [Driver README | 驱动说明](src/realman_arm_driver/README.md)
- [Software Requirements | 软件需求](docs/SRD.md)
- [Software Design | 软件设计](docs/SDD.md)
- [CAN Protocol | CAN协议](refs/jointMotorCAN_v1.0.md)

## License | 许可证

MIT License
