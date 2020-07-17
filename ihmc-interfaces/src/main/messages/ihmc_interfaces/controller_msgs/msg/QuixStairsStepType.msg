# This message is part of the Quix controller API.
# This message is used to notify the controller of the user's desired stairs step type

byte FLAT = 0
byte UP = 1
byte DOWN = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte desired_stairs_step_type 255