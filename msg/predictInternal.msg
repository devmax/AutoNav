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
AutoNav/filter_state prior_state

#posterior state after prediction
AutoNav/filter_state posterior_state
