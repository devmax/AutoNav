int32 timestamp
uint32 seq

#raw valocities given by IMU

float32 nav_vx
float32 nav_vy

#velocities after being rotated to world frame

float32 global_vx
float32 global_vy

<<<<<<< HEAD
AutoNav/filter_state prior_state

AutoNav/filter_state posterior_state
=======
#state and variances pre-observation

AutoNav/filter_state pre_state
AutoNav/filter_var pre_var

#state and variance post-observation

AutoNav/filter_state post_state
AutoNav/filter_var post_var
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
