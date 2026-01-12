#include "realman_arm_driver/can_protocol.hpp"

#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>

// CAN FD support
#ifndef CANFD_MTU
#define CANFD_MTU (sizeof(struct canfd_frame))
#endif

namespace realman_arm_driver
{

CanInterface::CanInterface(const std::string & interface_name)
  : interface_name_(interface_name)
{
}

CanInterface::~CanInterface()
{
  close();
}

bool CanInterface::open()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (socket_fd_ >= 0) {
    return true;  // Already open
  }

  // Create CAN FD socket
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return false;
  }

  // Enable CAN FD
  int canfd_on = 1;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on)) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Get interface index
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Bind to interface
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  return true;
}

void CanInterface::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanInterface::sendFrame(uint32_t id, const uint8_t * data, uint8_t len)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (socket_fd_ < 0) {
    return false;
  }

  struct canfd_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  
  frame.can_id = id;
  frame.len = len;
  frame.flags = CANFD_BRS;  // Bit rate switch enabled
  
  if (data && len > 0) {
    std::memcpy(frame.data, data, std::min(len, static_cast<uint8_t>(64)));
  }

  ssize_t nbytes = write(socket_fd_, &frame, CANFD_MTU);
  return nbytes == static_cast<ssize_t>(CANFD_MTU);
}

std::optional<CanFrame> CanInterface::receiveFrame(int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (socket_fd_ < 0) {
    return std::nullopt;
  }

  // Poll for data
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return std::nullopt;  // Timeout or error
  }

  // Read frame
  struct canfd_frame frame;
  ssize_t nbytes = read(socket_fd_, &frame, CANFD_MTU);
  
  if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
    return std::nullopt;
  }

  // Convert to our frame type
  CanFrame result;
  result.id = frame.can_id & CAN_EFF_MASK;  // Mask off flags
  result.len = frame.len;
  result.is_fd = (nbytes == static_cast<ssize_t>(CANFD_MTU));
  result.brs = (frame.flags & CANFD_BRS) != 0;
  std::memcpy(result.data, frame.data, std::min(frame.len, static_cast<uint8_t>(64)));

  return result;
}

std::optional<CanFrame> CanInterface::receiveFrameWithId(uint32_t expected_id, int timeout_ms)
{
  auto start = std::chrono::steady_clock::now();
  
  while (true) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    int remaining_ms = timeout_ms - 
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    
    if (remaining_ms <= 0) {
      return std::nullopt;
    }

    auto frame = receiveFrame(remaining_ms);
    if (!frame) {
      return std::nullopt;
    }

    if (frame->id == expected_id) {
      return frame;
    }
    // Discard frames with wrong ID and continue
  }
}

// ============================================================================
// High-level motor control functions
// ============================================================================

ServoFeedback CanInterface::sendPositionCommand(uint16_t joint_id, int32_t position_raw)
{
  ServoFeedback feedback;
  
  // Build command frame (4 bytes, little-endian int32)
  uint8_t data[4];
  data[0] = position_raw & 0xFF;
  data[1] = (position_raw >> 8) & 0xFF;
  data[2] = (position_raw >> 16) & 0xFF;
  data[3] = (position_raw >> 24) & 0xFF;

  uint32_t tx_id = joint_id + CanIdOffset::POSITION_CMD;
  uint32_t rx_id = joint_id + CanIdOffset::SERVO_FEEDBACK;

  if (!sendFrame(tx_id, data, 4)) {
    return feedback;
  }

  auto response = receiveFrameWithId(rx_id, 10);
  if (response && response->len >= 16) {
    feedback = parseServoFeedback(*response);
  }

  return feedback;
}

ServoFeedback CanInterface::sendVelocityCommand(uint16_t joint_id, int32_t velocity_raw)
{
  ServoFeedback feedback;
  
  uint8_t data[4];
  data[0] = velocity_raw & 0xFF;
  data[1] = (velocity_raw >> 8) & 0xFF;
  data[2] = (velocity_raw >> 16) & 0xFF;
  data[3] = (velocity_raw >> 24) & 0xFF;

  uint32_t tx_id = joint_id + CanIdOffset::VELOCITY_CMD;
  uint32_t rx_id = joint_id + CanIdOffset::SERVO_FEEDBACK;

  if (!sendFrame(tx_id, data, 4)) {
    return feedback;
  }

  auto response = receiveFrameWithId(rx_id, 10);
  if (response && response->len >= 16) {
    feedback = parseServoFeedback(*response);
  }

  return feedback;
}

ServoFeedback CanInterface::sendCurrentCommand(uint16_t joint_id, int32_t current_ma)
{
  ServoFeedback feedback;
  
  uint8_t data[4];
  data[0] = current_ma & 0xFF;
  data[1] = (current_ma >> 8) & 0xFF;
  data[2] = (current_ma >> 16) & 0xFF;
  data[3] = (current_ma >> 24) & 0xFF;

  uint32_t tx_id = joint_id + CanIdOffset::CURRENT_CMD;
  uint32_t rx_id = joint_id + CanIdOffset::SERVO_FEEDBACK;

  if (!sendFrame(tx_id, data, 4)) {
    return feedback;
  }

  auto response = receiveFrameWithId(rx_id, 10);
  if (response && response->len >= 16) {
    feedback = parseServoFeedback(*response);
  }

  return feedback;
}

JointStatus CanInterface::queryStatus(uint16_t joint_id)
{
  JointStatus status;

  uint32_t tx_id = joint_id + CanIdOffset::STATUS_QUERY;
  uint32_t rx_id = joint_id + CanIdOffset::STATUS_FEEDBACK;

  // Status query has 0 data bytes
  if (!sendFrame(tx_id, nullptr, 0)) {
    return status;
  }

  auto response = receiveFrameWithId(rx_id, 10);
  if (response && response->len >= 16) {
    status = parseStatusFeedback(*response);
  }

  return status;
}

bool CanInterface::writeRegister(uint16_t joint_id, uint8_t reg_addr, uint16_t value)
{
  uint8_t data[4];
  data[0] = static_cast<uint8_t>(CanCommand::CMD_WRITE);
  data[1] = reg_addr;
  data[2] = value & 0xFF;
  data[3] = (value >> 8) & 0xFF;

  uint32_t tx_id = joint_id;
  uint32_t rx_id = joint_id + CanIdOffset::RESPONSE;

  if (!sendFrame(tx_id, data, 4)) {
    return false;
  }

  auto response = receiveFrameWithId(rx_id, 10);
  if (!response || response->len < 4) {
    return false;
  }

  // Check response: CMD, INDEX, DATA (0x01 = success)
  return response->data[0] == static_cast<uint8_t>(CanCommand::CMD_WRITE) &&
         response->data[1] == reg_addr &&
         response->data[2] == 0x01;
}

std::vector<uint16_t> CanInterface::readRegisters(uint16_t joint_id, uint8_t reg_addr, uint8_t count)
{
  std::vector<uint16_t> result;

  uint8_t data[3];
  data[0] = static_cast<uint8_t>(CanCommand::CMD_READ);
  data[1] = reg_addr;
  data[2] = count;

  uint32_t tx_id = joint_id;
  uint32_t rx_id = joint_id + CanIdOffset::RESPONSE;

  if (!sendFrame(tx_id, data, 3)) {
    return result;
  }

  auto response = receiveFrameWithId(rx_id, 10);
  if (!response || response->len < 2 + count * 2) {
    return result;
  }

  // Parse response data (little-endian 16-bit values)
  for (uint8_t i = 0; i < count; ++i) {
    uint16_t val = response->data[2 + i * 2] | 
                   (response->data[3 + i * 2] << 8);
    result.push_back(val);
  }

  return result;
}

bool CanInterface::setEnabled(uint16_t joint_id, bool enabled)
{
  return writeRegister(joint_id, Register::SYS_ENABLE_DRIVER, enabled ? 1 : 0);
}

bool CanInterface::setMode(uint16_t joint_id, ControlMode mode)
{
  return writeRegister(joint_id, Register::TAG_WORK_MODE, static_cast<uint16_t>(mode));
}

bool CanInterface::setZero(uint16_t joint_id)
{
  return writeRegister(joint_id, Register::SYS_SET_ZERO_POS, 1);
}

bool CanInterface::clearErrors(uint16_t joint_id)
{
  return writeRegister(joint_id, Register::SYS_CLEAR_ERROR, 1);
}

bool CanInterface::clearIapFlag(uint16_t joint_id)
{
  return writeRegister(joint_id, Register::IAP_FLAG, 0);
}

// ============================================================================
// Private helper functions
// ============================================================================

ServoFeedback CanInterface::parseServoFeedback(const CanFrame & frame)
{
  ServoFeedback fb;
  
  if (frame.len < 16) {
    return fb;
  }

  // D0-D3: Current (int32, mA)
  fb.current_ma = static_cast<int32_t>(
    frame.data[0] | 
    (frame.data[1] << 8) | 
    (frame.data[2] << 16) | 
    (frame.data[3] << 24));

  // D4-D7: Velocity (int32, 0.02 RPM units)
  fb.velocity_raw = static_cast<int32_t>(
    frame.data[4] | 
    (frame.data[5] << 8) | 
    (frame.data[6] << 16) | 
    (frame.data[7] << 24));

  // D8-D11: Position (int32, 0.0001 degree units)
  fb.position_raw = static_cast<int32_t>(
    frame.data[8] | 
    (frame.data[9] << 8) | 
    (frame.data[10] << 16) | 
    (frame.data[11] << 24));

  // D12-D13: Enable state (uint16)
  fb.enabled = frame.data[12];

  // D14-D15: Error code (uint16)
  fb.error_code = frame.data[14] | (frame.data[15] << 8);

  fb.valid = true;
  return fb;
}

JointStatus CanInterface::parseStatusFeedback(const CanFrame & frame)
{
  JointStatus status;
  
  if (frame.len < 16) {
    return status;
  }

  // D0-D1: Error code
  status.error_code = frame.data[0] | (frame.data[1] << 8);

  // D2-D3: System voltage (0.01V units)
  status.voltage_raw = frame.data[2] | (frame.data[3] << 8);

  // D4-D5: System temperature (0.1°C units)
  status.temp_raw = frame.data[4] | (frame.data[5] << 8);

  // D6: Enable state
  status.enabled = frame.data[6];

  // D7: Brake state
  status.brake_state = frame.data[7];

  // D8-D11: Position (int32, 0.0001 degree units)
  status.position_raw = static_cast<int32_t>(
    frame.data[8] | 
    (frame.data[9] << 8) | 
    (frame.data[10] << 16) | 
    (frame.data[11] << 24));

  // D12-D15: Current (int32, mA)
  status.current_raw = static_cast<int32_t>(
    frame.data[12] | 
    (frame.data[13] << 8) | 
    (frame.data[14] << 16) | 
    (frame.data[15] << 24));

  status.valid = true;
  return status;
}

}  // namespace realman_arm_driver
