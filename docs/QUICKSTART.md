# Quick Start Guide | 快速启动指南

Complete step-by-step commands to bring up the RealMan arm driver on Jetson Orin.

在Jetson Orin上启动睿尔曼机械臂驱动的完整分步命令。

---

## Prerequisites | 前置条件

- Motors physically connected via CAN bus | 电机已通过CAN总线物理连接
- Motor IDs configured as: base_yaw=6, base_pitch=5, elbow=4 | 电机ID已配置为：base_yaw=6, base_pitch=5, elbow=4
- CAN termination resistors in place | CAN终端电阻已安装

---

## Step 1: Setup CAN FD Interface | 步骤1：设置CAN FD接口

```bash
# Bring down interface first (if already up) | 先关闭接口（如果已启动）
sudo ip link set can0 down 2>/dev/null

# Configure CAN FD: 1Mbps arbitration, 5Mbps data | 配置CAN FD：1Mbps仲裁，5Mbps数据
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on

# Bring up interface | 启动接口
sudo ip link set can0 up

# Verify configuration | 验证配置
ip -details link show can0
```

**Expected output should show | 预期输出应显示:**
- `bitrate 1000000`
- `dbitrate 5000000`
- `fd on`

---

## Step 2: Verify CAN Communication | 步骤2：验证CAN通信

```bash
# Monitor CAN bus (run in background, Ctrl+C to stop) | 监控CAN总线
candump can0 -x &

# Or with timeout | 或带超时
timeout 5 candump can0 -x
```

---

## Step 3: Scan for Motors | 步骤3：扫描电机

```bash
cd /home/nvidia/yanbo/armv1_ws
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --scan
```

**Expected output | 预期输出:**
```
Scanning for motors on can0 (IDs 1-30)...
Found motor ID 4: voltage=24.00V, temp=25.0°C, enabled=0
Found motor ID 5: voltage=24.00V, temp=25.0°C, enabled=0
Found motor ID 6: voltage=24.00V, temp=25.0°C, enabled=0
Total: 3 motor(s) found
```

**If no motors found | 如果未找到电机:**
- Check power to motors | 检查电机电源
- Check CAN wiring (CANH, CANL, GND) | 检查CAN接线
- Verify termination resistors | 验证终端电阻

---

## Step 4: Initialize All Motors (Required) | 步骤4：初始化所有电机（必须）

**IMPORTANT: This step is REQUIRED before motors will communicate!**

**重要：此步骤是电机通信前的必要操作！**

```bash
# Initialize all 3 motors at once (clears IAP flags and errors)
# 一次初始化所有3个电机（清除IAP标志和错误）
cd /home/nvidia/yanbo/armv1_ws
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --init-all

# Or specify custom motor IDs | 或指定自定义电机ID
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --init-all --motor-ids 4,5,6
```

**Expected output | 预期输出:**
```
============================================================
Initializing all motors | 初始化所有电机
============================================================
Motor IDs: [4, 5, 6]

--- Motor ID 4 ---
  [1/3] Clearing IAP flag... | 清除IAP标志...
        OK | 成功
  [2/3] Clearing errors... | 清除错误...
        OK | 成功
  [3/3] Verifying status... | 验证状态...
        OK - Voltage: 24.0V, Temp: 25.0°C

--- Motor ID 5 ---
  ...

============================================================
Result: 3/3 motors initialized successfully
============================================================

All motors ready! You can now run the ROS2 driver.
```

**Alternative: Initialize motors individually | 备选：逐个初始化电机:**

```bash
# Clear IAP flags (required before any communication) | 清除IAP标志（通信前必须）
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --clear-iap
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 5 --clear-iap
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --clear-iap

# Clear any existing errors | 清除任何现有错误
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --clear-errors
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 5 --clear-errors
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --clear-errors
```

---

## Step 5: Check Motor Status | 步骤5：检查电机状态

```bash
# Check each motor | 检查每个电机
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --status
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 5 --status
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --status
```

---

## Step 6: Test Single Motor (Optional) | 步骤6：测试单个电机（可选）

## Step 6: Test Single Motor (Optional) | 步骤6：测试单个电机（可选）

```bash
# Enable motor 6 (base_yaw) | 使能电机6（底座偏航）
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --enable

# Check it's enabled | 检查是否已使能
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --status

# Disable when done testing | 测试完成后禁用
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --disable
```

---

## Step 7: Launch ROS2 Driver | 步骤7：启动ROS2驱动

### Terminal 1 - Driver | 终端1 - 驱动

```bash
cd /home/nvidia/yanbo/armv1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realman_arm_driver arm_driver.launch.py
```

**Expected startup log | 预期启动日志:**
```
[INFO] [realman_arm_driver]: CAN interface: can0
[INFO] [realman_arm_driver]: Joints: base_yaw(6), base_pitch(5), elbow(4)
[INFO] [realman_arm_driver]: Control rate: 50.0 Hz
[INFO] [realman_arm_driver]: Driver initialized successfully
```

---

## Step 8: Test ROS2 Interface | 步骤8：测试ROS2接口

### Terminal 2 - Commands | 终端2 - 命令

```bash
# Setup environment | 设置环境
cd /home/nvidia/yanbo/armv1_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Monitor joint states | 监控关节状态
ros2 topic echo /realman_arm_driver/joint_states

# Monitor arm status | 监控机械臂状态
ros2 topic echo /realman_arm_driver/status

# Enable all motors | 使能所有电机
ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: true}"

# Check topic list | 查看话题列表
ros2 topic list | grep realman

# Check service list | 查看服务列表
ros2 service list | grep realman
```

---

## Step 9: Send Position Commands | 步骤9：发送位置命令

```bash
# Send position command (degrees, all zeros = current position) | 发送位置命令（单位：度，全零=当前位置）
ros2 topic pub --once /realman_arm_driver/joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_yaw', 'base_pitch', 'elbow'], position: [0.0, 0.0, 0.0]}"

# Move base_yaw 10 degrees | 移动base_yaw 10度
ros2 topic pub --once /realman_arm_driver/joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_yaw', 'base_pitch', 'elbow'], position: [10.0, 0.0, 0.0]}"

# Return to zero | 返回零位
ros2 topic pub --once /realman_arm_driver/joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_yaw', 'base_pitch', 'elbow'], position: [0.0, 0.0, 0.0]}"
```

Note: position commands are in degrees, and the driver limits each command step to 20° (larger moves are split automatically).  
注意：位置指令单位为度，驱动会将单次指令步长限制为 20°（更大的角度会自动拆分）。

---

## Step 10: Shutdown | 步骤10：关闭

```bash
# Disable all motors | 禁用所有电机
ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: false}"

# Then Ctrl+C the driver in Terminal 1 | 然后在终端1按Ctrl+C停止驱动
```

---

## Troubleshooting | 故障排除

### Problem: No motors found during scan | 问题：扫描时未找到电机

```bash
# Check CAN interface is up | 检查CAN接口是否启动
ip link show can0

# Check for any CAN traffic | 检查CAN总线是否有数据
candump can0 -x

# Try standard CAN (non-FD) for debugging | 尝试标准CAN进行调试
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### Problem: Motor ID conflict | 问题：电机ID冲突

If multiple motors have the same ID (factory default is ID=1):

如果多个电机ID相同（出厂默认ID=1）：

```bash
# Connect only ONE motor at a time | 每次只连接一个电机
# Then change its ID | 然后更改ID
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --current-id 1 --new-id 6

# Power cycle motor, then connect next one | 给电机断电重启，然后连接下一个
```

### Problem: Motors have errors | 问题：电机有错误

```bash
# Check error codes | 检查错误代码
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --status

# Clear errors | 清除错误
python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 6 --clear-errors
```

### Problem: ROS2 driver fails to start | 问题：ROS2驱动启动失败

```bash
# Check if CAN interface exists | 检查CAN接口是否存在
ip link show can0

# Rebuild if needed | 如需要则重新编译
cd /home/nvidia/yanbo/armv1_ws
colcon build --packages-select realman_arm_driver
source install/setup.bash
```

---

## Quick Reference Commands | 快速参考命令

| Action 操作 | Command 命令 |
|------------|-------------|
| Setup CAN FD | `sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set can0 up` |
| **Init all motors** | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --init-all` |
| Scan motors | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --scan` |
| Motor status | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --status` |
| Clear IAP | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --clear-iap` |
| Clear errors | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --clear-errors` |
| Enable motor | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --enable` |
| Disable motor | `python3 src/realman_arm_driver/scripts/motor_setup.py -i can0 --id 4 --disable` |
| Launch driver | `ros2 launch realman_arm_driver arm_driver.launch.py` |
| ROS enable | `ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: true}"` |
| ROS disable | `ros2 service call /realman_arm_driver/enable std_srvs/srv/SetBool "{data: false}"` |
| Monitor joints | `ros2 topic echo /realman_arm_driver/joint_states` |

---

## Motor ID Reference | 电机ID参考

| Joint 关节 | Name 名称 | CAN ID | Torque 扭矩 |
|------------|-----------|--------|-------------|
| 1 | base_yaw | 6 | 10 Nm |
| 2 | base_pitch | 5 | 30 Nm |
| 3 | elbow | 4 | 10 Nm |

---

## CAN Protocol Reference | CAN协议参考

| Command 命令 | TX ID | RX ID | Description 描述 |
|--------------|-------|-------|------------------|
| Register R/W | `motor_id` | `motor_id + 0x100` | Read/write registers |
| Position servo | `motor_id + 0x200` | `motor_id + 0x500` | Position control |
| Velocity servo | `motor_id + 0x300` | `motor_id + 0x500` | Velocity control |
| Current servo | `motor_id + 0x400` | `motor_id + 0x500` | Current/torque control |
| Status query | `motor_id + 0x600` | `motor_id + 0x700` | Query motor status |

Example for motor ID 6:
- Position command TX: 0x206, Feedback RX: 0x506
- Status query TX: 0x606, Response RX: 0x706
