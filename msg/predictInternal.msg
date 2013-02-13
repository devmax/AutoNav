#time when message was published
int32 timestamp

int32 timeSpanMicros

geometry_msgs/Twist activeControlInfo

int32 useControlGains
int32 controlValid

# gains because of given Control Commands
float32 rollControlGain
float32 pitchControlGain
float32 yawSpeedControlGain

#Forces project on horizontal axes due to tilt
float32 forceX
float32 forceY

#Velocity gain due to forces on horizontal axes
float32 vx_gain
float32 vy_gain
float32 vz_gain

#prior state before prediction
float32 roll_pre
float32 pitch_pre
float32 yaw_pre
float32 dyaw_pre
float32 x_pre
float32 dx_pre
float32 y_pre
float32 dy_pre
float32 z_pre
float32 dz_pre

#posterior state after prediction
float32 roll_post
float32 pitch_post
float32 yaw_post
float32 dyaw_post
float32 x_post
float32 dx_post
float32 y_post
float32 dy_post
float32 z_post
float32 dz_post
