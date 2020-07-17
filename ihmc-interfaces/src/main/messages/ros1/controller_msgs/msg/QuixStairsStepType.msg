# This message is part of the Quix controller API.
# This message is used to notify the controller of the user's desired stairs step type

int8 FLAT=0

int8 UP=1

int8 DOWN=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 desired_stairs_step_type


