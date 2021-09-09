# Used by Kusbegi Flight Task and Kusbegi Control


uint64 timestamp			# time since system start (microseconds)


uint8 status

uint8 NONE = 0
uint8 CIRCLE_DONE = 1
uint8 CIRCLE_STARTED = 2
uint8 CIRCLE_STOPPED = 3
uint8 USING_TARGET = 4
uint8 DESCENDING = 5
uint8 DESCENDING_DONE = 6
uint8 RED_LANDING = 7
uint8 RED_LANDED = 8

