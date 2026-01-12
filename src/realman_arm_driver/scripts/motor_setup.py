#!/usr/bin/env python3
"""
RealMan Motor Setup Utility | 睿尔曼电机设置工具

This utility helps configure RealMan joint motors via CAN FD.
本工具用于通过CAN FD配置睿尔曼关节电机。

Usage | 用法:
    python3 motor_setup.py --interface can0 --scan
    python3 motor_setup.py --interface can0 --current-id 1 --new-id 11
    python3 motor_setup.py --interface can0 --id 11 --status

Author: RecomoArm Team
Date: 2026-01-12
"""

import argparse
import socket
import struct
import time
import sys

# CAN FD socket constants
CAN_RAW = 1
CAN_FD_FRAME = 0x0800000

# Register addresses | 寄存器地址
REG_SYS_ID = 0x01
REG_SYS_FW_VERSION = 0x03
REG_SYS_ERROR = 0x04
REG_SYS_VOLTAGE = 0x05
REG_SYS_TEMP = 0x06
REG_SYS_ENABLE = 0x0A
REG_SAVE_TO_FLASH = 0x0C
REG_SET_ZERO = 0x0E
REG_CLEAR_ERROR = 0x0F
REG_WORK_MODE = 0x30
REG_IAP_FLAG = 0x49

# Command types | 指令类型
CMD_READ = 0x01
CMD_WRITE = 0x02

# CAN ID offsets | CAN ID偏移
OFFSET_RESPONSE = 0x100
OFFSET_STATUS_QUERY = 0x600
OFFSET_STATUS_RESPONSE = 0x700


class CanInterface:
    """CAN FD interface wrapper | CAN FD接口封装"""
    
    def __init__(self, interface: str):
        self.interface = interface
        self.sock = None
    
    def open(self) -> bool:
        """Open CAN socket | 打开CAN套接字"""
        try:
            # Create CAN socket
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
            
            # Enable CAN FD
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            
            # Bind to interface
            self.sock.bind((self.interface,))
            
            # Set timeout
            self.sock.settimeout(0.1)
            
            return True
        except Exception as e:
            print(f"Error opening CAN interface: {e}")
            print(f"打开CAN接口失败: {e}")
            return False
    
    def close(self):
        """Close CAN socket | 关闭CAN套接字"""
        if self.sock:
            self.sock.close()
            self.sock = None
    
    def send_frame(self, can_id: int, data: bytes) -> bool:
        """Send CAN FD frame | 发送CAN FD帧"""
        try:
            # CAN FD frame format: can_id (4) + len (1) + flags (1) + res (2) + data (64)
            flags = 0x01  # CANFD_BRS
            frame = struct.pack("=IB3x", can_id, len(data)) + data.ljust(64, b'\x00')
            self.sock.send(frame)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def receive_frame(self, timeout: float = 0.1) -> tuple:
        """Receive CAN FD frame | 接收CAN FD帧"""
        try:
            self.sock.settimeout(timeout)
            frame = self.sock.recv(72)  # CAN FD frame size
            can_id = struct.unpack("=I", frame[:4])[0] & 0x1FFFFFFF
            length = frame[4]
            data = frame[8:8+length]
            return can_id, data
        except socket.timeout:
            return None, None
        except Exception as e:
            print(f"Receive error: {e}")
            return None, None


def write_register(can: CanInterface, motor_id: int, reg: int, value: int) -> bool:
    """Write to motor register | 写入电机寄存器"""
    data = bytes([CMD_WRITE, reg, value & 0xFF, (value >> 8) & 0xFF])
    
    if not can.send_frame(motor_id, data):
        return False
    
    # Wait for response
    expected_id = motor_id + OFFSET_RESPONSE
    rx_id, rx_data = can.receive_frame(0.1)
    
    if rx_id == expected_id and rx_data and len(rx_data) >= 3:
        if rx_data[0] == CMD_WRITE and rx_data[1] == reg and rx_data[2] == 0x01:
            return True
    
    return False


def read_register(can: CanInterface, motor_id: int, reg: int, count: int = 1) -> list:
    """Read from motor register | 读取电机寄存器"""
    data = bytes([CMD_READ, reg, count])
    
    if not can.send_frame(motor_id, data):
        return None
    
    # Wait for response
    expected_id = motor_id + OFFSET_RESPONSE
    rx_id, rx_data = can.receive_frame(0.1)
    
    if rx_id == expected_id and rx_data and len(rx_data) >= 2 + count * 2:
        values = []
        for i in range(count):
            val = rx_data[2 + i*2] | (rx_data[3 + i*2] << 8)
            values.append(val)
        return values
    
    return None


def query_status(can: CanInterface, motor_id: int) -> dict:
    """Query motor status | 查询电机状态"""
    # Send status query (ID + 0x600, no data)
    tx_id = motor_id + OFFSET_STATUS_QUERY
    
    if not can.send_frame(tx_id, b''):
        return None
    
    # Wait for response (ID + 0x700, 16 bytes)
    expected_id = motor_id + OFFSET_STATUS_RESPONSE
    rx_id, rx_data = can.receive_frame(0.1)
    
    if rx_id == expected_id and rx_data and len(rx_data) >= 16:
        error_code = rx_data[0] | (rx_data[1] << 8)
        voltage = (rx_data[2] | (rx_data[3] << 8)) * 0.01
        temp = (rx_data[4] | (rx_data[5] << 8)) * 0.1
        enabled = rx_data[6]
        brake = rx_data[7]
        position = struct.unpack('<i', rx_data[8:12])[0] * 0.0001
        current = struct.unpack('<i', rx_data[12:16])[0]
        
        return {
            'error_code': error_code,
            'voltage': voltage,
            'temperature': temp,
            'enabled': enabled,
            'brake': brake,
            'position': position,
            'current': current
        }
    
    return None


def scan_motors(can: CanInterface, id_range: range = range(1, 31)) -> list:
    """Scan for motors on CAN bus | 扫描CAN总线上的电机"""
    found = []
    
    print("Scanning for motors... | 扫描电机中...")
    print(f"ID range: {id_range.start}-{id_range.stop-1}")
    print()
    
    for motor_id in id_range:
        status = query_status(can, motor_id)
        if status:
            found.append(motor_id)
            print(f"  Found motor ID {motor_id}: voltage={status['voltage']:.2f}V, "
                  f"temp={status['temperature']:.1f}°C, enabled={status['enabled']}")
            print(f"  发现电机 ID {motor_id}: 电压={status['voltage']:.2f}V, "
                  f"温度={status['temperature']:.1f}°C, 使能={status['enabled']}")
    
    if not found:
        print("No motors found | 未发现电机")
    else:
        print(f"\nTotal: {len(found)} motor(s) found | 共发现 {len(found)} 个电机")
    
    return found


def get_motor_status(can: CanInterface, motor_id: int):
    """Get and display motor status | 获取并显示电机状态"""
    print(f"Querying motor ID {motor_id}... | 查询电机 ID {motor_id}...")
    
    status = query_status(can, motor_id)
    
    if status:
        print(f"\n{'='*50}")
        print(f"Motor ID {motor_id} Status | 电机 ID {motor_id} 状态")
        print(f"{'='*50}")
        print(f"  Voltage | 电压:        {status['voltage']:.2f} V")
        print(f"  Temperature | 温度:    {status['temperature']:.1f} °C")
        print(f"  Enabled | 使能:        {'Yes 是' if status['enabled'] else 'No 否'}")
        print(f"  Brake | 抱闸:          {'Open 打开' if status['brake'] else 'Closed 关闭'}")
        print(f"  Position | 位置:       {status['position']:.4f} °")
        print(f"  Current | 电流:        {status['current']} mA")
        print(f"  Error Code | 错误代码: 0x{status['error_code']:04X}")
        
        if status['error_code'] != 0:
            print(f"\n  Errors | 错误:")
            errors = {
                0x0001: "FOC rate high | FOC频率过高",
                0x0002: "Over voltage | 过压",
                0x0004: "Under voltage | 欠压",
                0x0008: "Over temperature | 过温",
                0x0010: "Startup failed | 启动失败",
                0x0020: "Encoder error | 编码器错误",
                0x0040: "Over current | 过流",
                0x0080: "Software error | 软件错误",
                0x0100: "Temp sensor error | 温度传感器错误",
                0x0200: "Position limit | 位置超限",
                0x0400: "Invalid ID | ID非法",
                0x0800: "Tracking error | 跟踪误差超限",
                0x1000: "Current sensor | 电流检测错误",
                0x2000: "Brake failure | 抱闸失败",
                0x4000: "Position step | 位置指令阶跃",
                0x8000: "Multi-turn lost | 多圈丢数",
            }
            for code, desc in errors.items():
                if status['error_code'] & code:
                    print(f"    - {desc}")
        print(f"{'='*50}")
    else:
        print(f"No response from motor ID {motor_id}")
        print(f"电机 ID {motor_id} 无响应")


def change_motor_id(can: CanInterface, current_id: int, new_id: int) -> bool:
    """Change motor ID | 更改电机ID"""
    
    if new_id < 1 or new_id > 30:
        print(f"Error: New ID must be 1-30, got {new_id}")
        print(f"错误：新ID必须在1-30之间，输入为 {new_id}")
        return False
    
    print(f"Changing motor ID from {current_id} to {new_id}...")
    print(f"将电机ID从 {current_id} 更改为 {new_id}...")
    
    # Step 1: Verify motor exists
    status = query_status(can, current_id)
    if not status:
        print(f"Error: No motor found at ID {current_id}")
        print(f"错误：在ID {current_id} 未发现电机")
        return False
    
    print(f"  Found motor at ID {current_id}")
    print(f"  在ID {current_id} 发现电机")
    
    # Step 2: Disable motor first
    print("  Disabling motor... | 禁用电机...")
    if not write_register(can, current_id, REG_SYS_ENABLE, 0):
        print("  Warning: Could not disable motor | 警告：无法禁用电机")
    time.sleep(0.01)
    
    # Step 3: Write new ID
    print(f"  Writing new ID {new_id}... | 写入新ID {new_id}...")
    if not write_register(can, current_id, REG_SYS_ID, new_id):
        print("  Error: Failed to write new ID | 错误：写入新ID失败")
        return False
    
    # Step 4: Save to Flash
    print("  Saving to Flash... | 保存到Flash...")
    time.sleep(0.05)  # Wait 50ms as per protocol
    if not write_register(can, current_id, REG_SAVE_TO_FLASH, 1):
        print("  Warning: Could not save to Flash | 警告：无法保存到Flash")
    
    time.sleep(0.1)
    
    print()
    print("="*50)
    print("SUCCESS! | 成功！")
    print("="*50)
    print(f"Motor ID changed from {current_id} to {new_id}")
    print(f"电机ID已从 {current_id} 更改为 {new_id}")
    print()
    print("IMPORTANT: You MUST power cycle the motor now!")
    print("重要：您必须现在给电机断电重启！")
    print("="*50)
    
    return True


def clear_errors(can: CanInterface, motor_id: int) -> bool:
    """Clear motor errors | 清除电机错误"""
    print(f"Clearing errors for motor ID {motor_id}...")
    print(f"清除电机 ID {motor_id} 的错误...")
    
    if write_register(can, motor_id, REG_CLEAR_ERROR, 1):
        print("  Errors cleared | 错误已清除")
        return True
    else:
        print("  Failed to clear errors | 清除错误失败")
        return False


def set_zero(can: CanInterface, motor_id: int) -> bool:
    """Set current position as zero | 将当前位置设为零位"""
    print(f"Setting zero position for motor ID {motor_id}...")
    print(f"设置电机 ID {motor_id} 的零位...")
    
    if write_register(can, motor_id, REG_SET_ZERO, 1):
        print("  Zero position set | 零位已设置")
        # Save to flash
        time.sleep(0.05)
        write_register(can, motor_id, REG_SAVE_TO_FLASH, 1)
        print("  Saved to Flash | 已保存到Flash")
        return True
    else:
        print("  Failed to set zero | 设置零位失败")
        return False


def enable_motor(can: CanInterface, motor_id: int, enable: bool) -> bool:
    """Enable or disable motor | 使能或禁用电机"""
    action = "Enabling" if enable else "Disabling"
    action_cn = "使能" if enable else "禁用"
    
    print(f"{action} motor ID {motor_id}...")
    print(f"{action_cn}电机 ID {motor_id}...")
    
    if write_register(can, motor_id, REG_SYS_ENABLE, 1 if enable else 0):
        print(f"  Motor {'enabled | 已使能' if enable else 'disabled | 已禁用'}")
        return True
    else:
        print(f"  Failed | 失败")
        return False


def clear_iap_flag(can: CanInterface, motor_id: int) -> bool:
    """Clear IAP flag (required before normal operation) | 清除IAP标志"""
    print(f"Clearing IAP flag for motor ID {motor_id}...")
    print(f"清除电机 ID {motor_id} 的IAP标志...")
    
    if write_register(can, motor_id, REG_IAP_FLAG, 0):
        print("  IAP flag cleared | IAP标志已清除")
        return True
    else:
        print("  Failed to clear IAP flag | 清除IAP标志失败")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='RealMan Motor Setup Utility | 睿尔曼电机设置工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples | 示例:
  Scan for motors | 扫描电机:
    %(prog)s -i can0 --scan

  Get motor status | 获取电机状态:
    %(prog)s -i can0 --id 1 --status

  Change motor ID | 更改电机ID:
    %(prog)s -i can0 --current-id 1 --new-id 11

  Clear errors | 清除错误:
    %(prog)s -i can0 --id 11 --clear-errors

  Set zero position | 设置零位:
    %(prog)s -i can0 --id 11 --set-zero

  Enable motor | 使能电机:
    %(prog)s -i can0 --id 11 --enable

  Disable motor | 禁用电机:
    %(prog)s -i can0 --id 11 --disable

  Clear IAP flag | 清除IAP标志:
    %(prog)s -i can0 --id 11 --clear-iap

  Setup all 3 motors (from factory default) | 设置全部3个电机（从出厂默认）:
    # Connect motor 1 only, then: | 只连接电机1，然后:
    %(prog)s -i can0 --current-id 1 --new-id 11
    # Power cycle, connect motor 2 only, then: | 断电重启，只连接电机2，然后:
    %(prog)s -i can0 --current-id 1 --new-id 12
    # Power cycle, connect motor 3 only, then: | 断电重启，只连接电机3，然后:
    %(prog)s -i can0 --current-id 1 --new-id 13
"""
    )
    
    parser.add_argument('-i', '--interface', default='can0',
                        help='CAN interface name (default: can0) | CAN接口名称')
    parser.add_argument('--scan', action='store_true',
                        help='Scan for motors on CAN bus | 扫描CAN总线上的电机')
    parser.add_argument('--id', type=int,
                        help='Motor ID for operations | 操作的电机ID')
    parser.add_argument('--current-id', type=int,
                        help='Current motor ID (for ID change) | 当前电机ID（用于更改ID）')
    parser.add_argument('--new-id', type=int,
                        help='New motor ID (for ID change) | 新电机ID（用于更改ID）')
    parser.add_argument('--status', action='store_true',
                        help='Get motor status | 获取电机状态')
    parser.add_argument('--clear-errors', action='store_true',
                        help='Clear motor errors | 清除电机错误')
    parser.add_argument('--set-zero', action='store_true',
                        help='Set current position as zero | 将当前位置设为零位')
    parser.add_argument('--enable', action='store_true',
                        help='Enable motor | 使能电机')
    parser.add_argument('--disable', action='store_true',
                        help='Disable motor | 禁用电机')
    parser.add_argument('--clear-iap', action='store_true',
                        help='Clear IAP flag | 清除IAP标志')
    
    args = parser.parse_args()
    
    # Open CAN interface
    can = CanInterface(args.interface)
    if not can.open():
        print(f"\nMake sure CAN interface is up | 请确保CAN接口已启动:")
        print(f"  sudo ip link set {args.interface} type can bitrate 1000000 dbitrate 5000000 fd on")
        print(f"  sudo ip link set {args.interface} up")
        sys.exit(1)
    
    print(f"CAN interface '{args.interface}' opened | CAN接口 '{args.interface}' 已打开")
    print()
    
    try:
        # Handle commands
        if args.scan:
            scan_motors(can)
        
        elif args.current_id is not None and args.new_id is not None:
            change_motor_id(can, args.current_id, args.new_id)
        
        elif args.id is not None:
            if args.status:
                get_motor_status(can, args.id)
            elif args.clear_errors:
                clear_errors(can, args.id)
            elif args.set_zero:
                set_zero(can, args.id)
            elif args.enable:
                enable_motor(can, args.id, True)
            elif args.disable:
                enable_motor(can, args.id, False)
            elif args.clear_iap:
                clear_iap_flag(can, args.id)
            else:
                # Default: show status
                get_motor_status(can, args.id)
        
        else:
            parser.print_help()
    
    finally:
        can.close()


if __name__ == '__main__':
    main()
