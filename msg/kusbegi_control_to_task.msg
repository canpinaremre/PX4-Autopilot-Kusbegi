# Used by Kusbegi Flight Task and Kusbegi Control


uint64 timestamp			# time since system start (microseconds)

uint8 mission

uint8 MISSON_EMPTY = 1
uint8 MISSON_DO_CIRCLE = 2
uint8 MISSON_STOP_CIRCLE = 3
uint8 MISSON_USE_TARGET = 4
uint8 MISSON_DESCEND = 5
uint8 MISSON_RED = 6

float32 param1
float32 param2
