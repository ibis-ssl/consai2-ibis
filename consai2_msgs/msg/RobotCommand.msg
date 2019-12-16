# robot command for consai2_control and consai2_sender

uint8   robot_id
# forward/back, m/s
float64 vel_surge
# left/right, m/s
float64 vel_sway
# rotation, rad/s
float64 vel_angular

# target robot pose
float64 target_pos_x
float64 target_pos_y
float64 target_theta

# current robot pose
float64 pos_x
float64 pos_y
float64 theta

# ball position
float64 ball_x
float64 ball_y


bool local_mode_enable

# 0.0 ~ 1.0
float64 kick_power
# True/False
bool    chip_enable
# 0.0 ~ 1.0
float64 dribble_power

# default:unused
bool    test_flag
# default:unused
float64 test_param

