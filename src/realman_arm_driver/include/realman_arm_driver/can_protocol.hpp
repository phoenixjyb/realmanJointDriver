#ifndef REALMAN_ARM_DRIVER__CAN_PROTOCOL_HPP_
#define REALMAN_ARM_DRIVER__CAN_PROTOCOL_HPP_

#include <string>
#include <optional>
#include <vector>
#include <memory>
#include <mutex>

#include "realman_arm_driver/types.hpp"

namespace realman_arm_driver
{

/**
 * @brief CAN FD interface for RealMan joint motors
 * 
 * Implements the CAN FD protocol for communication with RealMan joint motors.
 * Uses SocketCAN for Linux CAN interface.
 */
class CanInterface
{
public:
  /**
   * @brief Constructor
   * @param interface_name SocketCAN interface name (e.g., "can0")
   */
  explicit CanInterface(const std::string & interface_name);

  /**
   * @brief Destructor - closes socket if open
   */
  ~CanInterface();

  // Disable copy
  CanInterface(const CanInterface &) = delete;
  CanInterface & operator=(const CanInterface &) = delete;

  /**
   * @brief Open the CAN interface
   * @return true if successful
   */
  bool open();

  /**
   * @brief Close the CAN interface
   */
  void close();

  /**
   * @brief Check if interface is open
   */
  bool isOpen() const { return socket_fd_ >= 0; }

  /**
   * @brief Send a CAN FD frame
   * @param id CAN ID
   * @param data Data bytes
   * @param len Data length
   * @return true if successful
   */
  bool sendFrame(uint32_t id, const uint8_t * data, uint8_t len);

  /**
   * @brief Receive a CAN FD frame
   * @param timeout_ms Timeout in milliseconds
   * @return Frame if received, nullopt on timeout/error
   */
  std::optional<CanFrame> receiveFrame(int timeout_ms);

  /**
   * @brief Receive a frame with specific ID
   * @param expected_id Expected CAN ID
   * @param timeout_ms Timeout in milliseconds
   * @return Frame if received with matching ID
   */
  std::optional<CanFrame> receiveFrameWithId(uint32_t expected_id, int timeout_ms);

  // =========================================================================
  // High-level motor control functions
  // =========================================================================

  /**
   * @brief Send position servo command and receive feedback
   * @param joint_id Motor CAN ID
   * @param position_raw Position in 0.0001 degree units
   * @return Servo feedback
   */
  ServoFeedback sendPositionCommand(uint16_t joint_id, int32_t position_raw);

  /**
   * @brief Send velocity servo command and receive feedback
   * @param joint_id Motor CAN ID
   * @param velocity_raw Velocity in 0.002 RPM units
   * @return Servo feedback
   */
  ServoFeedback sendVelocityCommand(uint16_t joint_id, int32_t velocity_raw);

  /**
   * @brief Send current servo command and receive feedback
   * @param joint_id Motor CAN ID
   * @param current_ma Current in mA
   * @return Servo feedback
   */
  ServoFeedback sendCurrentCommand(uint16_t joint_id, int32_t current_ma);

  /**
   * @brief Query joint status
   * @param joint_id Motor CAN ID
   * @return Joint status
   */
  JointStatus queryStatus(uint16_t joint_id);

  /**
   * @brief Write to a register
   * @param joint_id Motor CAN ID
   * @param reg_addr Register address
   * @param value 16-bit value to write
   * @return true if successful
   */
  bool writeRegister(uint16_t joint_id, uint8_t reg_addr, uint16_t value);

  /**
   * @brief Read from registers
   * @param joint_id Motor CAN ID
   * @param reg_addr Starting register address
   * @param count Number of registers to read
   * @return Vector of 16-bit values, empty on error
   */
  std::vector<uint16_t> readRegisters(uint16_t joint_id, uint8_t reg_addr, uint8_t count);

  /**
   * @brief Set motor enable state
   * @param joint_id Motor CAN ID
   * @param enabled true to enable, false to disable
   * @return true if successful
   */
  bool setEnabled(uint16_t joint_id, bool enabled);

  /**
   * @brief Set control mode
   * @param joint_id Motor CAN ID
   * @param mode Control mode
   * @return true if successful
   */
  bool setMode(uint16_t joint_id, ControlMode mode);

  /**
   * @brief Set current position as zero
   * @param joint_id Motor CAN ID
   * @return true if successful
   */
  bool setZero(uint16_t joint_id);

  /**
   * @brief Clear errors
   * @param joint_id Motor CAN ID
   * @return true if successful
   */
  bool clearErrors(uint16_t joint_id);

  /**
   * @brief Clear IAP flag (required before normal operation)
   * @param joint_id Motor CAN ID
   * @return true if successful
   */
  bool clearIapFlag(uint16_t joint_id);

  /**
   * @brief Clear IAP flag with detailed error reporting
   * @param joint_id Motor CAN ID
   * @param error_msg Output string for error details
   * @return true if successful
   */
  bool clearIapFlagWithDetails(uint16_t joint_id, std::string & error_msg);

  /**
   * @brief Get interface name
   */
  const std::string & getInterfaceName() const { return interface_name_; }

private:
  std::string interface_name_;
  int socket_fd_{-1};
  mutable std::mutex mutex_;  // Thread safety for socket operations

  /**
   * @brief Parse servo feedback from received frame
   */
  ServoFeedback parseServoFeedback(const CanFrame & frame);

  /**
   * @brief Parse status feedback from received frame
   */
  JointStatus parseStatusFeedback(const CanFrame & frame);
};

}  // namespace realman_arm_driver

#endif  // REALMAN_ARM_DRIVER__CAN_PROTOCOL_HPP_
