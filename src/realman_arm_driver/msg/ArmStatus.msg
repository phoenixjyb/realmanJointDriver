# Arm-wide status message
std_msgs/Header header

# Overall state
bool all_enabled                    # All joints enabled
bool any_errors                     # Any joint has errors

# Per-joint status
realman_arm_driver/JointStatus[] joints

# Control mode (1=current, 2=velocity, 3=position)
uint8 control_mode

# Constants for control mode
uint8 MODE_CURRENT=1
uint8 MODE_VELOCITY=2
uint8 MODE_POSITION=3
