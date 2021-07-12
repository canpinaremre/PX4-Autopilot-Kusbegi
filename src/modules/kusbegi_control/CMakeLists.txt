
px4_add_module(
	MODULE modules__kusbegi_control
	MAIN kusbegi_control
	COMPILE_FLAGS
	SRCS
		kusbegi_control.cpp
	DEPENDS
		# px4_work_queue
)
