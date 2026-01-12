#ifndef REALMAN_ARM_DRIVER__TYPES_HPP_
#define REALMAN_ARM_DRIVER__TYPES_HPP_

#include <cstdint>
#include <string>
#include <cmath>

namespace realman_arm_driver
{

/**
 * @brief Control modes matching motor protocol
 */
enum class ControlMode : uint8_t
{
  OPEN_LOOP = 0,
  CURRENT = 1,
  VELOCITY = 2,
  POSITION = 3
};

/**
 * @brief CAN command types
 */
enum class CanCommand : uint8_t
{
  CMD_READ = 0x01,
  CMD_WRITE = 0x02
};

/**
 * @brief Memory register addresses
 */
namespace Register
{
  constexpr uint8_t SYS_ID = 0x01;
  constexpr uint8_t SYS_FW_VERSION = 0x03;
  constexpr uint8_t SYS_ERROR = 0x04;
  constexpr uint8_t SYS_VOLTAGE = 0x05;
  constexpr uint8_t SYS_TEMP = 0x06;
  constexpr uint8_t SYS_REDU_RATIO = 0x07;
  constexpr uint8_t SYS_ENABLE_DRIVER = 0x0A;
  constexpr uint8_t SYS_ENABLE_ON_POWER = 0x0B;
  constexpr uint8_t SYS_SAVE_TO_FLASH = 0x0C;
  constexpr uint8_t SYS_SET_ZERO_POS = 0x0E;
  constexpr uint8_t SYS_CLEAR_ERROR = 0x0F;
  constexpr uint8_t CUR_CURRENT_L = 0x10;
  constexpr uint8_t CUR_CURRENT_H = 0x11;
  constexpr uint8_t CUR_SPEED_L = 0x12;
  constexpr uint8_t CUR_SPEED_H = 0x13;
  constexpr uint8_t CUR_POSITION_L = 0x14;
  constexpr uint8_t CUR_POSITION_H = 0x15;
  constexpr uint8_t TAG_WORK_MODE = 0x30;
  constexpr uint8_t TAG_CURRENT_L = 0x32;
  constexpr uint8_t TAG_CURRENT_H = 0x33;
  constexpr uint8_t TAG_SPEED_L = 0x34;
  constexpr uint8_t TAG_SPEED_H = 0x35;
  constexpr uint8_t TAG_POSITION_L = 0x36;
  constexpr uint8_t TAG_POSITION_H = 0x37;
  constexpr uint8_t LIT_MAX_CURRENT = 0x40;
  constexpr uint8_t LIT_MAX_SPEED = 0x41;
  constexpr uint8_t LIT_MAX_ACC = 0x42;
  constexpr uint8_t LIT_MAX_DEC = 0x43;
  constexpr uint8_t LIT_MIN_POSITION_L = 0x44;
  constexpr uint8_t LIT_MIN_POSITION_H = 0x45;
  constexpr uint8_t LIT_MAX_POSITION_L = 0x46;
  constexpr uint8_t LIT_MAX_POSITION_H = 0x47;
  constexpr uint8_t IAP_FLAG = 0x49;
}  // namespace Register

/**
 * @brief CAN ID offsets for different command types
 */
namespace CanIdOffset
{
  constexpr uint16_t RESPONSE = 0x100;
  constexpr uint16_t POSITION_CMD = 0x200;
  constexpr uint16_t VELOCITY_CMD = 0x300;
  constexpr uint16_t CURRENT_CMD = 0x400;
  constexpr uint16_t SERVO_FEEDBACK = 0x500;
  constexpr uint16_t STATUS_QUERY = 0x600;
  constexpr uint16_t STATUS_FEEDBACK = 0x700;
}  // namespace CanIdOffset

/**
 * @brief Error code bit definitions
 */
namespace ErrorCode
{
  constexpr uint16_t FOC_RATE_HIGH = 0x0001;
  constexpr uint16_t OVER_VOLTAGE = 0x0002;
  constexpr uint16_t UNDER_VOLTAGE = 0x0004;
  constexpr uint16_t OVER_TEMPERATURE = 0x0008;
  constexpr uint16_t STARTUP_FAILED = 0x0010;
  constexpr uint16_t ENCODER_ERROR = 0x0020;
  constexpr uint16_t OVER_CURRENT = 0x0040;
  constexpr uint16_t SOFTWARE_ERROR = 0x0080;
  constexpr uint16_t TEMP_SENSOR_ERROR = 0x0100;
  constexpr uint16_t POSITION_LIMIT = 0x0200;
  constexpr uint16_t INVALID_ID = 0x0400;
  constexpr uint16_t TRACKING_ERROR = 0x0800;
  constexpr uint16_t CURRENT_SENSOR_ERROR = 0x1000;
  constexpr uint16_t BRAKE_FAILURE = 0x2000;
  constexpr uint16_t POSITION_STEP = 0x4000;
  constexpr uint16_t MULTI_TURN_LOST = 0x8000;
}  // namespace ErrorCode

/**
 * @brief Joint configuration
 */
struct JointConfig
{
  std::string name;
  uint16_t can_id;
  double position_min;   // radians
  double position_max;   // radians
  double velocity_max;   // rad/s
  double current_max;    // mA
  double torque_rating;  // Nm
};

/**
 * @brief Servo feedback from position/velocity/current commands
 */
struct ServoFeedback
{
  int32_t current_ma{0};      // Current in mA
  int32_t velocity_raw{0};    // Velocity in 0.02 RPM units
  int32_t position_raw{0};    // Position in 0.0001 degree units
  uint8_t enabled{0};         // Enable state
  uint16_t error_code{0};     // Error code
  bool valid{false};          // Whether feedback is valid
};

/**
 * @brief Status query feedback
 */
struct JointStatus
{
  uint16_t error_code{0};     // Error code
  uint16_t voltage_raw{0};    // Voltage in 0.01V units
  uint16_t temp_raw{0};       // Temperature in 0.1°C units
  uint8_t enabled{0};         // Enable state
  uint8_t brake_state{0};     // Brake state
  int32_t position_raw{0};    // Position in 0.0001 degree units
  int32_t current_raw{0};     // Current in mA
  bool valid{false};          // Whether status is valid
};

/**
 * @brief CAN frame structure
 */
struct CanFrame
{
  uint32_t id{0};
  uint8_t data[64]{0};
  uint8_t len{0};
  bool is_fd{true};
  bool brs{true};  // Bit rate switch
};

// ============================================================================
// Unit Conversion Functions
// ============================================================================

/**
 * @brief Convert radians to raw position (0.0001 degree units)
 */
inline int32_t radiansToRaw(double radians)
{
  return static_cast<int32_t>(radians * 180.0 / M_PI * 10000.0);
}

/**
 * @brief Convert raw position to radians
 */
inline double rawToRadians(int32_t raw)
{
  return static_cast<double>(raw) / 10000.0 * M_PI / 180.0;
}

/**
 * @brief Convert rad/s to raw velocity command (0.002 RPM units)
 */
inline int32_t radPerSecToRawCmd(double rad_per_sec)
{
  double rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
  return static_cast<int32_t>(rpm / 0.002);
}

/**
 * @brief Convert raw velocity feedback (0.02 RPM units) to rad/s
 */
inline double rawFeedbackToRadPerSec(int32_t raw)
{
  double rpm = static_cast<double>(raw) * 0.02;
  return rpm * 2.0 * M_PI / 60.0;
}

/**
 * @brief Convert raw voltage (0.01V units) to volts
 */
inline double rawToVolts(uint16_t raw)
{
  return static_cast<double>(raw) * 0.01;
}

/**
 * @brief Convert raw temperature (0.1°C units) to Celsius
 */
inline double rawToCelsius(uint16_t raw)
{
  return static_cast<double>(raw) * 0.1;
}

/**
 * @brief Get human-readable error description
 */
inline std::string getErrorDescription(uint16_t error_code)
{
  std::string desc;
  if (error_code & ErrorCode::FOC_RATE_HIGH) desc += "FOC_RATE_HIGH ";
  if (error_code & ErrorCode::OVER_VOLTAGE) desc += "OVER_VOLTAGE ";
  if (error_code & ErrorCode::UNDER_VOLTAGE) desc += "UNDER_VOLTAGE ";
  if (error_code & ErrorCode::OVER_TEMPERATURE) desc += "OVER_TEMP ";
  if (error_code & ErrorCode::STARTUP_FAILED) desc += "STARTUP_FAIL ";
  if (error_code & ErrorCode::ENCODER_ERROR) desc += "ENCODER_ERR ";
  if (error_code & ErrorCode::OVER_CURRENT) desc += "OVER_CURRENT ";
  if (error_code & ErrorCode::SOFTWARE_ERROR) desc += "SW_ERROR ";
  if (error_code & ErrorCode::TEMP_SENSOR_ERROR) desc += "TEMP_SENSOR ";
  if (error_code & ErrorCode::POSITION_LIMIT) desc += "POS_LIMIT ";
  if (error_code & ErrorCode::INVALID_ID) desc += "INVALID_ID ";
  if (error_code & ErrorCode::TRACKING_ERROR) desc += "TRACKING_ERR ";
  if (error_code & ErrorCode::CURRENT_SENSOR_ERROR) desc += "CUR_SENSOR ";
  if (error_code & ErrorCode::BRAKE_FAILURE) desc += "BRAKE_FAIL ";
  if (error_code & ErrorCode::POSITION_STEP) desc += "POS_STEP ";
  if (error_code & ErrorCode::MULTI_TURN_LOST) desc += "MULTI_TURN ";
  return desc.empty() ? "OK" : desc;
}

}  // namespace realman_arm_driver

#endif  // REALMAN_ARM_DRIVER__TYPES_HPP_
