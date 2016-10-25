# Error flag constants
int32 DXL_INSTRUCTION_ERROR   = 64
int32 DXL_OVERLOAD_ERROR      = 32
int32 DXL_CHECKSUM_ERROR      = 16
int32 DXL_RANGE_ERROR         = 8
int32 DXL_OVERHEATING_ERROR   = 4
int32 DXL_ANGLE_LIMIT_ERROR   = 2
int32 DXL_INPUT_VOLTAGE_ERROR = 1
int32 DXL_NO_ERROR            = 0


float64 timestamp   # motor state is at this time
int32 id            # motor id
int32 flags         # error flags
int32 goal          # commanded position (in encoder units)
int32 position      # current position (in encoder units)
int32 error         # difference between current and goal positions
int32 speed         # current speed (0.111 rpm per unit)
float64 load        # current load - ratio of applied torque over maximum torque
float64 voltage     # current voltage (V)
int32 temperature   # current temperature (degrees Celsius)
bool moving         # whether the motor is currently in motion
