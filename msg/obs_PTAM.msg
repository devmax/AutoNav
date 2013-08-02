int32 timestamp
uint32 seq

int32 rejected

# the observation
float32 x
float32 y
float32 z
float32 roll
float32 pitch
float32 yaw

#the variances

float32 varX
float32 varY
float32 varZ
float32 varRoll
float32 varPitch
float32 varYaw

#the prior
AutoNav/filter_state prior_state
AutoNav/filter_var prior_var

#the posterior
AutoNav/filter_state posterior_state
AutoNav/filter_var posterior_var