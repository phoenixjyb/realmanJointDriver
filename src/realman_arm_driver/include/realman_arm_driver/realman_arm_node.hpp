#ifndef REALMAN_ARM_DRIVER__REALMAN_ARM_NODE_HPP_
#define REALMAN_ARM_DRIVER__REALMAN_ARM_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "realman_arm_driver/types.hpp"
#include "realman_arm_driver/can_protocol.hpp"
#include "realman_arm_driver/msg/arm_status.hpp"
#include "realman_arm_driver/srv/set_mode.hpp"

namespace realman_arm_driver
{

/**
 * @brief ROS2 node for RealMan arm driver
 */
class RealmanArmNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit RealmanArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~RealmanArmNode() override;

private:
  // =========================================================================
  // Initialization
  // =========================================================================
  
  /**
   * @brief Declare and load parameters
   */
  void declareParameters();

  /**
   * @brief Load parameters into member variables
   */
  void loadParameters();

  /**
   * @brief Initialize CAN interface
   */
  bool initializeCan();

  /**
   * @brief Initialize motors (clear IAP flags, set mode)
   */
  bool initializeMotors();

  // =========================================================================
  // Callbacks
  // =========================================================================

  /**
   * @brief Control loop callback - runs at control_rate_hz
   */
  void controlLoopCallback();

  /**
   * @brief Status loop callback - runs at status_rate_hz
   */
  void statusLoopCallback();

  /**
   * @brief Joint command callback
   */
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // =========================================================================
  // Service handlers
  // =========================================================================

  /**
   * @brief Enable/disable service handler
   */
  void enableServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Set mode service handler
   */
  void setModeServiceCallback(
    const std::shared_ptr<srv::SetMode::Request> request,
    std::shared_ptr<srv::SetMode::Response> response);

  /**
   * @brief Set zero service handler
   */
  void setZeroServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Clear errors service handler
   */
  void clearErrorsServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // =========================================================================
  // Helper functions
  // =========================================================================

  /**
   * @brief Clamp position to joint limits
   */
  double clampPosition(size_t joint_idx, double position);

  /**
   * @brief Clamp velocity to limits
   */
  double clampVelocity(size_t joint_idx, double velocity);

  /**
   * @brief Clamp current to limits
   */
  double clampCurrent(size_t joint_idx, double current);

  /**
   * @brief Emergency stop - disable all motors
   */
  void emergencyStop();

  // =========================================================================
  // Member variables
  // =========================================================================

  // CAN interface
  std::unique_ptr<CanInterface> can_interface_;

  // Joint configuration
  std::vector<JointConfig> joints_;
  size_t num_joints_{0};

  // State
  ControlMode control_mode_{ControlMode::POSITION};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> initialized_{false};
  int consecutive_errors_{0};

  // Targets (protected by mutex)
  std::mutex target_mutex_;
  std::vector<double> target_positions_;   // radians
  std::vector<double> target_velocities_;  // rad/s
  std::vector<double> target_currents_;    // mA

  // Current state (from feedback)
  std::vector<double> current_positions_;   // radians
  std::vector<double> current_velocities_;  // rad/s
  std::vector<double> current_currents_;    // mA (as effort in A)
  std::vector<uint16_t> error_codes_;

  // Parameters
  std::string can_interface_name_;
  double control_rate_hz_{50.0};
  double status_rate_hz_{10.0};
  int can_timeout_ms_{10};
  int max_consecutive_errors_{5};
  bool enable_on_startup_{false};
  bool clear_errors_on_startup_{true};
  bool set_iap_flag_on_startup_{true};

  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<msg::ArmStatus>::SharedPtr arm_status_pub_;

  // ROS2 subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;

  // ROS2 services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_zero_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_errors_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace realman_arm_driver

#endif  // REALMAN_ARM_DRIVER__REALMAN_ARM_NODE_HPP_
