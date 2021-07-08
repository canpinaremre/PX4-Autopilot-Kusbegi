# Used by Kusbegi Flight Task

#Kusbegi target spesific message

uint64 timestamp			# time since system start (microseconds)

uint8 KUSBEGI_TARGET_TYPE_RED = 1
uint8 KUSBEGI_TARGET_TYPE_POOL = 2

uint8 kusbegi_target_type

float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
float32 yaw			# yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw

float32 x			# local position setpoint in m in NED
float32 y			# local position setpoint in m in NED
float32 z			# local position setpoint in m in NED

uint8 taget_value
uint8 target_confident
