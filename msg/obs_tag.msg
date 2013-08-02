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

#the prior
AutoNav/filter_state prior_state

#the posterior
AutoNav/filter_state posterior_state