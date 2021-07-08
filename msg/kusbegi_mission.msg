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

