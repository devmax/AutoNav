int32 timestamp
uint32 seq

int32 rejected

float32 roll_raw
float32 pitch_raw
float32 yaw_raw

# the observation
float32 x
float32 y
float32 z
float32 roll
float32 pitch
float32 yaw

#the prior
float32 x_pre
float32 y_pre
float32 z_pre
float32 dx_pre
float32 dy_pre
float32 dz_pre
float32 roll_pre
float32 pitch_pre
float32 yaw_pre
float32 dyaw_pre
float32 varx_pre
float32 vary_pre
float32 vardx_pre
float32 vardy_pre

#the posterior
float32 x_post
float32 y_post
float32 z_post
float32 dx_post
float32 dy_post
float32 dz_post
float32 roll_post
float32 pitch_post
float32 yaw_post
float32 dyaw_post
float32 varx_post
float32 vary_post
float32 vardx_post
float32 vardy_post