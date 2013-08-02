int32 timestamp
uint32 seq

#indicate the result of the yaw observation routine
uint32 code 

#measurements
float32 roll
float32 pitch

float32 baselineY_IMU
float32 baselineY_Filter
float32 navYaw
float32 observedYaw

#state and variances pre-observation

AutoNav/filter_state pre_state
AutoNav/filter_var pre_var

#state and variance post-observation

AutoNav/filter_state post_state
AutoNav/filter_var post_var

