# One command from the debug GUI to the controller server.
#
# The GUI never calls a service itself. Service calls block until the drone
# answers - and if the drone is off or wedged, that wait is unbounded, which
# would freeze the whole window including the emergency stop. So the GUI
# publishes this (publishing never blocks) and the controller server, which
# already owns the service clients, does the actual call.

# Which command this message carries.
uint8 CMD_NONE    = 0
uint8 CMD_TAKEOFF = 1
uint8 CMD_LAND    = 2
uint8 CMD_GOTO    = 3
uint8 CMD_ESTOP   = 4

std_msgs/Header header

uint8 command
string drone_name        # who this is for; controller ignores other names

# GOTO parameters. Ignored by every other command.
float32 x
float32 y
float32 z
float32 yaw              # radians
float32 duration         # seconds to reach the target

# TAKEOFF parameter. Ignored by every other command.
float32 height           # metres above the floor
