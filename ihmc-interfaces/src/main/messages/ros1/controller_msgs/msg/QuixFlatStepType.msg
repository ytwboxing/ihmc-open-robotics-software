# This message is part of the Quix controller API.
# This message is used to notify the controller of the user's desired flat step type

int8 SQUARE=0

int8 SHORT_FORWARD=1

int8 MEDIUM_FORWARD=2

int8 LONG_FORWARD=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 desired_flat_step_type


