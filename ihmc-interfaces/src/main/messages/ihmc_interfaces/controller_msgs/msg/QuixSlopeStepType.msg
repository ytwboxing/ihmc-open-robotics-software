# This message is part of the Quix controller API.
# This message is used to notify the controller of the user's desired slope step type

byte SLOPE_FLAT = 0
byte SLOPE_UP = 1
byte SLOPE_DOWN = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte desired_slope_step_type 255