# Used by Kusbegi Flight Task

uint64 timestamp			# time since system start (microseconds)

uint8 KUSBEGI_STATE_IDLE = 1
uint8 KUSBEGI_STATE_GOTO_WP = 2
uint8 KUSBEGI_STATE_TAKE_WATER = 3
uint8 KUSBEGI_STATE_DUMP_WATER = 4
uint8 KUSBEGI_STATE_GO_STRAIGHT = 5
uint8 KUSBEGI_STATE_SEARCH_RED = 6
uint8 KUSBEGI_STATE_SEARCH_POOL = 7
uint8 KUSBEGI_STATE_UNKOWN = 8

bool mission_valid
uint64 mission_start_timestamp 		#To calculate timeout

uint8 kusbegi_state

float32 param1			# Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param2			# Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param3			# Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param4			# Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
float64 param5			# Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
float64 param6			# Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param7			# Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.


uint8 MODULE_COMMANDER = 1
uint8 MODULE_KUSBEGI_CONTROL = 2
uint8 FT_KUSBEGI = 3
uint8 publisher
