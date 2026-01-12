#include "realman_arm_driver/realman_arm_node.hpp"

#include <chrono>
#include <functional>

namespace realman_arm_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

RealmanArmNode::RealmanArmNode(const rclcpp::NodeOptions & options)
  : Node("realman_arm_driver", options)
{
  RCLCPP_INFO(get_logger(), "Initializing RealMan Arm Driver...");

  // Initialize
  declareParameters();
  loadParameters();

  // Resize state vectors
  target_positions_.resize(num_joints_, 0.0);
  target_velocities_.resize(num_joints_, 0.0);
  target_currents_.resize(num_joints_, 0.0);
  current_positions_.resize(num_joints_, 0.0);
  current_velocities_.resize(num_joints_, 0.0);
  current_currents_.resize(num_joints_, 0.0);
  error_codes_.resize(num_joints_, 0);

  // Create publishers
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_states", 10);
  arm_status_pub_ = create_publisher<msg::ArmStatus>(
    "~/status", 10);

  // Create subscribers
  joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "~/joint_commands", 10,
    std::bind(&RealmanArmNode::jointCommandCallback, this, _1));

  // Create services
  enable_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/enable",
    std::bind(&RealmanArmNode::enableServiceCallback, this, _1, _2));
  
  set_mode_srv_ = create_service<srv::SetMode>(
    "~/set_mode",
    std::bind(&RealmanArmNode::setModeServiceCallback, this, _1, _2));
  
  set_zero_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/set_zero",
    std::bind(&RealmanArmNode::setZeroServiceCallback, this, _1, _2));
  
  clear_errors_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_errors",
    std::bind(&RealmanArmNode::clearErrorsServiceCallback, this, _1, _2));

  // Initialize CAN
  if (!initializeCan()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize CAN interface!");
    return;
  }

  // Initialize motors
  if (!initializeMotors()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize motors!");
    return;
  }

  // Create timers
  auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
    std::bind(&RealmanArmNode::controlLoopCallback, this));

  auto status_period = std::chrono::duration<double>(1.0 / status_rate_hz_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
    std::bind(&RealmanArmNode::statusLoopCallback, this));

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "RealMan Arm Driver initialized successfully!");
  RCLCPP_INFO(get_logger(), "  Control rate: %.1f Hz", control_rate_hz_);
  RCLCPP_INFO(get_logger(), "  Status rate: %.1f Hz", status_rate_hz_);
  RCLCPP_INFO(get_logger(), "  Joints: %zu", num_joints_);
  for (const auto & joint : joints_) {
    RCLCPP_INFO(get_logger(), "    - %s (ID: %d)", joint.name.c_str(), joint.can_id);
  }
}

RealmanArmNode::~RealmanArmNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down RealMan Arm Driver...");
  
  // Disable all motors before shutdown
  if (can_interface_ && can_interface_->isOpen()) {
    for (const auto & joint : joints_) {
      can_interface_->setEnabled(joint.can_id, false);
    }
  }
}

// ============================================================================
// Initialization
// ============================================================================

void RealmanArmNode::declareParameters()
{
  // CAN interface
  declare_parameter("can_interface", "can0");
  
  // Control rates
  declare_parameter("control_rate_hz", 50.0);
  declare_parameter("status_rate_hz", 10.0);
  
  // Communication
  declare_parameter("can_timeout_ms", 10);
  declare_parameter("max_consecutive_errors", 5);
  
  // Joint configuration
  declare_parameter("joint_names", std::vector<std::string>{"base_yaw", "base_pitch", "elbow"});
  declare_parameter("joint_ids", std::vector<int64_t>{306, 305, 304});
  declare_parameter("joint_torques", std::vector<double>{10.0, 30.0, 10.0});
  
  // Limits
  declare_parameter("position_limits_min", std::vector<double>{-3.14159, -1.5708, -2.3562});
  declare_parameter("position_limits_max", std::vector<double>{3.14159, 1.5708, 2.3562});
  declare_parameter("velocity_limits", std::vector<double>{1.0, 1.0, 1.0});
  declare_parameter("current_limits", std::vector<double>{10000.0, 20000.0, 10000.0});
  
  // Initial settings
  declare_parameter("default_mode", 3);
  declare_parameter("enable_on_startup", false);
  declare_parameter("clear_errors_on_startup", true);
  declare_parameter("set_iap_flag_on_startup", true);
}

void RealmanArmNode::loadParameters()
{
  can_interface_name_ = get_parameter("can_interface").as_string();
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  status_rate_hz_ = get_parameter("status_rate_hz").as_double();
  can_timeout_ms_ = static_cast<int>(get_parameter("can_timeout_ms").as_int());
  max_consecutive_errors_ = static_cast<int>(get_parameter("max_consecutive_errors").as_int());
  enable_on_startup_ = get_parameter("enable_on_startup").as_bool();
  clear_errors_on_startup_ = get_parameter("clear_errors_on_startup").as_bool();
  set_iap_flag_on_startup_ = get_parameter("set_iap_flag_on_startup").as_bool();

  auto joint_names = get_parameter("joint_names").as_string_array();
  auto joint_ids = get_parameter("joint_ids").as_integer_array();
  auto joint_torques = get_parameter("joint_torques").as_double_array();
  auto pos_min = get_parameter("position_limits_min").as_double_array();
  auto pos_max = get_parameter("position_limits_max").as_double_array();
  auto vel_limits = get_parameter("velocity_limits").as_double_array();
  auto cur_limits = get_parameter("current_limits").as_double_array();

  int default_mode = static_cast<int>(get_parameter("default_mode").as_int());
  control_mode_ = static_cast<ControlMode>(default_mode);

  num_joints_ = joint_names.size();
  joints_.resize(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i) {
    joints_[i].name = joint_names[i];
    joints_[i].can_id = static_cast<uint16_t>(joint_ids[i]);
    joints_[i].torque_rating = joint_torques[i];
    joints_[i].position_min = pos_min[i];
    joints_[i].position_max = pos_max[i];
    joints_[i].velocity_max = vel_limits[i];
    joints_[i].current_max = cur_limits[i];
  }
}

bool RealmanArmNode::initializeCan()
{
  can_interface_ = std::make_unique<CanInterface>(can_interface_name_);
  
  if (!can_interface_->open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN interface: %s", can_interface_name_.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "CAN interface '%s' opened successfully", can_interface_name_.c_str());
  return true;
}

bool RealmanArmNode::initializeMotors()
{
  for (const auto & joint : joints_) {
    RCLCPP_INFO(get_logger(), "Initializing joint '%s' (ID: %d)...", 
                joint.name.c_str(), joint.can_id);

    // Clear IAP flag (required per protocol)
    if (set_iap_flag_on_startup_) {
      if (!can_interface_->clearIapFlag(joint.can_id)) {
        RCLCPP_WARN(get_logger(), "Failed to clear IAP flag for joint %s", joint.name.c_str());
      }
    }

    // Clear errors
    if (clear_errors_on_startup_) {
      if (!can_interface_->clearErrors(joint.can_id)) {
        RCLCPP_WARN(get_logger(), "Failed to clear errors for joint %s", joint.name.c_str());
      }
    }

    // Set mode
    if (!can_interface_->setMode(joint.can_id, control_mode_)) {
      RCLCPP_WARN(get_logger(), "Failed to set mode for joint %s", joint.name.c_str());
    }

    // Query initial status
    auto status = can_interface_->queryStatus(joint.can_id);
    if (status.valid) {
      RCLCPP_INFO(get_logger(), "  Joint %s: voltage=%.2fV, temp=%.1f°C, enabled=%d, errors=0x%04X",
                  joint.name.c_str(),
                  rawToVolts(status.voltage_raw),
                  rawToCelsius(status.temp_raw),
                  status.enabled,
                  status.error_code);
    } else {
      RCLCPP_WARN(get_logger(), "  Failed to query status for joint %s", joint.name.c_str());
    }
  }

  // Enable motors if configured
  if (enable_on_startup_) {
    for (const auto & joint : joints_) {
      can_interface_->setEnabled(joint.can_id, true);
    }
    enabled_ = true;
    RCLCPP_INFO(get_logger(), "Motors enabled on startup");
  }

  return true;
}

// ============================================================================
// Callbacks
// ============================================================================

void RealmanArmNode::controlLoopCallback()
{
  if (!initialized_ || !can_interface_ || !can_interface_->isOpen()) {
    return;
  }

  // Get current targets
  std::vector<double> targets_pos, targets_vel, targets_cur;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    targets_pos = target_positions_;
    targets_vel = target_velocities_;
    targets_cur = target_currents_;
  }

  [[maybe_unused]] bool any_error = false;

  // Send commands to each joint and collect feedback
  for (size_t i = 0; i < num_joints_; ++i) {
    ServoFeedback fb;

    if (enabled_) {
      switch (control_mode_) {
        case ControlMode::POSITION: {
          double clamped = clampPosition(i, targets_pos[i]);
          int32_t raw = radiansToRaw(clamped);
          fb = can_interface_->sendPositionCommand(joints_[i].can_id, raw);
          break;
        }
        case ControlMode::VELOCITY: {
          double clamped = clampVelocity(i, targets_vel[i]);
          int32_t raw = radPerSecToRawCmd(clamped);
          fb = can_interface_->sendVelocityCommand(joints_[i].can_id, raw);
          break;
        }
        case ControlMode::CURRENT: {
          double clamped = clampCurrent(i, targets_cur[i]);
          int32_t raw = static_cast<int32_t>(clamped);
          fb = can_interface_->sendCurrentCommand(joints_[i].can_id, raw);
          break;
        }
        default:
          break;
      }
    } else {
      // When not enabled, just query status to get current state
      auto status = can_interface_->queryStatus(joints_[i].can_id);
      if (status.valid) {
        fb.position_raw = status.position_raw;
        fb.current_ma = status.current_raw;
        fb.velocity_raw = 0;  // Status doesn't include velocity
        fb.error_code = status.error_code;
        fb.enabled = status.enabled;
        fb.valid = true;
      }
    }

    if (fb.valid) {
      current_positions_[i] = rawToRadians(fb.position_raw);
      current_velocities_[i] = rawFeedbackToRadPerSec(fb.velocity_raw);
      current_currents_[i] = fb.current_ma / 1000.0;  // Convert to A
      error_codes_[i] = fb.error_code;
      consecutive_errors_ = 0;

      if (fb.error_code != 0) {
        any_error = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Joint %s error: 0x%04X (%s)", 
          joints_[i].name.c_str(), fb.error_code,
          getErrorDescription(fb.error_code).c_str());
      }
    } else {
      consecutive_errors_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "No feedback from joint %s", joints_[i].name.c_str());
    }
  }

  // Check for communication failure
  if (consecutive_errors_ > max_consecutive_errors_) {
    RCLCPP_ERROR(get_logger(), "Too many consecutive CAN errors, triggering emergency stop");
    emergencyStop();
  }

  // Publish joint states
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = now();
  joint_state_msg.name.reserve(num_joints_);
  joint_state_msg.position.reserve(num_joints_);
  joint_state_msg.velocity.reserve(num_joints_);
  joint_state_msg.effort.reserve(num_joints_);

  for (size_t i = 0; i < num_joints_; ++i) {
    joint_state_msg.name.push_back(joints_[i].name);
    joint_state_msg.position.push_back(current_positions_[i]);
    joint_state_msg.velocity.push_back(current_velocities_[i]);
    joint_state_msg.effort.push_back(current_currents_[i]);
  }

  joint_state_pub_->publish(joint_state_msg);
}

void RealmanArmNode::statusLoopCallback()
{
  if (!initialized_ || !can_interface_ || !can_interface_->isOpen()) {
    return;
  }

  msg::ArmStatus status_msg;
  status_msg.header.stamp = now();
  status_msg.control_mode = static_cast<uint8_t>(control_mode_);
  status_msg.all_enabled = enabled_.load();
  status_msg.any_errors = false;

  for (size_t i = 0; i < num_joints_; ++i) {
    auto status = can_interface_->queryStatus(joints_[i].can_id);
    
    msg::JointStatus joint_status;
    joint_status.header.stamp = now();
    joint_status.name = joints_[i].name;
    joint_status.can_id = joints_[i].can_id;

    if (status.valid) {
      joint_status.enabled = status.enabled != 0;
      joint_status.error_code = status.error_code;
      joint_status.voltage = rawToVolts(status.voltage_raw);
      joint_status.temperature = rawToCelsius(status.temp_raw);
      joint_status.current = status.current_raw / 1000.0f;
      joint_status.position = rawToRadians(status.position_raw);
      joint_status.velocity = current_velocities_[i];  // From servo feedback

      if (status.error_code != 0) {
        status_msg.any_errors = true;
      }
    }

    status_msg.joints.push_back(joint_status);
  }

  arm_status_pub_->publish(status_msg);
}

void RealmanArmNode::jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_mutex_);

  // Map incoming joint names to our indices
  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (size_t j = 0; j < num_joints_; ++j) {
      if (msg->name[i] == joints_[j].name) {
        // Update targets based on what's provided
        if (i < msg->position.size()) {
          target_positions_[j] = msg->position[i];
        }
        if (i < msg->velocity.size()) {
          target_velocities_[j] = msg->velocity[i];
        }
        if (i < msg->effort.size()) {
          target_currents_[j] = msg->effort[i] * 1000.0;  // A to mA
        }
        break;
      }
    }
  }
}

// ============================================================================
// Service handlers
// ============================================================================

void RealmanArmNode::enableServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  bool success = true;
  for (const auto & joint : joints_) {
    if (!can_interface_->setEnabled(joint.can_id, request->data)) {
      success = false;
      RCLCPP_ERROR(get_logger(), "Failed to %s joint %s", 
                   request->data ? "enable" : "disable", joint.name.c_str());
    }
  }

  if (success) {
    enabled_ = request->data;
    response->success = true;
    response->message = request->data ? "Motors enabled" : "Motors disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Failed to set enable state for some joints";
  }
}

void RealmanArmNode::setModeServiceCallback(
  const std::shared_ptr<srv::SetMode::Request> request,
  std::shared_ptr<srv::SetMode::Response> response)
{
  if (request->mode < 1 || request->mode > 3) {
    response->success = false;
    response->message = "Invalid mode. Use 1=current, 2=velocity, 3=position";
    return;
  }

  ControlMode new_mode = static_cast<ControlMode>(request->mode);
  bool success = true;

  for (const auto & joint : joints_) {
    if (!can_interface_->setMode(joint.can_id, new_mode)) {
      success = false;
      RCLCPP_ERROR(get_logger(), "Failed to set mode for joint %s", joint.name.c_str());
    }
  }

  if (success) {
    control_mode_ = new_mode;
    response->success = true;
    response->message = "Mode set to " + std::to_string(request->mode);
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Failed to set mode for some joints";
  }
}

void RealmanArmNode::setZeroServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool success = true;

  for (const auto & joint : joints_) {
    if (!can_interface_->setZero(joint.can_id)) {
      success = false;
      RCLCPP_ERROR(get_logger(), "Failed to set zero for joint %s", joint.name.c_str());
    }
  }

  if (success) {
    response->success = true;
    response->message = "Zero position set for all joints";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Failed to set zero for some joints";
  }
}

void RealmanArmNode::clearErrorsServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  bool success = true;

  for (const auto & joint : joints_) {
    if (!can_interface_->clearErrors(joint.can_id)) {
      success = false;
      RCLCPP_ERROR(get_logger(), "Failed to clear errors for joint %s", joint.name.c_str());
    }
  }

  if (success) {
    response->success = true;
    response->message = "Errors cleared for all joints";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Failed to clear errors for some joints";
  }
}

// ============================================================================
// Helper functions
// ============================================================================

double RealmanArmNode::clampPosition(size_t joint_idx, double position)
{
  return std::clamp(position, joints_[joint_idx].position_min, joints_[joint_idx].position_max);
}

double RealmanArmNode::clampVelocity(size_t joint_idx, double velocity)
{
  return std::clamp(velocity, -joints_[joint_idx].velocity_max, joints_[joint_idx].velocity_max);
}

double RealmanArmNode::clampCurrent(size_t joint_idx, double current)
{
  return std::clamp(current, -joints_[joint_idx].current_max, joints_[joint_idx].current_max);
}

void RealmanArmNode::emergencyStop()
{
  RCLCPP_ERROR(get_logger(), "EMERGENCY STOP TRIGGERED!");
  
  for (const auto & joint : joints_) {
    can_interface_->setEnabled(joint.can_id, false);
  }
  
  enabled_ = false;
}

}  // namespace realman_arm_driver
