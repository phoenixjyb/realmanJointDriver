# Individual joint status
std_msgs/Header header

string name              # Joint name
uint16 can_id            # CAN ID

# Motor state
bool enabled             # Motor enabled
uint16 error_code        # Error code (0 = no error)

# Electrical
float32 voltage          # System voltage (V)
float32 temperature      # System temperature (°C)
float32 current          # Current (A)

# Position
float64 position         # Current position (rad)
float64 position_deg     # Current position (deg)
float64 velocity         # Current velocity (rad/s)
