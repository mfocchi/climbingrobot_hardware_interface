# Timestamp so PlotJuggler can use ROS time
std_msgs/Header header

# Raw inputs
bool    brake                              # 1 if engaged
float32 current                            # A (ibus)
float32 motor_torque                       # Nm (as reported by ODrive)
int32   syncronous_roller_raw_wrapped      # counts [0..CPR)
float32 motor_position                     # revs in [0..1)

# Derived (you just computed these)
float32 motor_speed_rev_s                  # rev/s
float32 motor_speed_rad_s                  # rad/s
float32 sync_roller_speed_rev_s            # rev/s
float32 sync_roller_speed_rad_s            # rad/s
float32 rope_speed_m_s                     # m/s (roller linear)
