# This message is part of the Quix controller API.
# This message is used to allow the crutch to communicate a desired change in behavior.

uint32 sequence_id

bool user_enable

bool rewiggle

bool start_behavior

controller_msgs/QuixMotionStateMessage requested_motion_state

bool execute_behavior

controller_msgs/QuixFlatStepType desired_flat_step_type

bool desired_continuous_walk

controller_msgs/QuixSlopeStepType desired_slope_step_type

controller_msgs/QuixStairsStepType desired_stairs_step_type


