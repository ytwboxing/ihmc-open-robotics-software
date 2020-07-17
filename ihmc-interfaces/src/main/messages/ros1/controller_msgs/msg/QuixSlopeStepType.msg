# This message is part of the Quix controller API.
# This message is used to notify the controller of the user's desired slope step type

int8 SLOPE_UP=0

int8 SLOPE_DOWN=1

int8 SLOPE_FLAT=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 desired_slope_step_type


