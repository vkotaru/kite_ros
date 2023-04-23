# Command msg 

# control mode flags
uint8 MODE_PASS_THROUGH = 0
uint8 MODE_ATTITUDE = 1
uint8 MODE_ATTITUDE_RATE = 2
uint8 MODE_THRUST_YAW = 3
uint8 MODE_THRUST_YAW_RATE = 4
uint8 MODE_POSITION = 5
uint8 MODE_POSITION_SPLINE = 6

# message
uint8 mode
float64 thrust
float64[] yaw
geometry_msgs/Vector3[] values
kite_msgs/Spline spline
