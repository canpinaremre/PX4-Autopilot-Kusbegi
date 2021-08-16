from dronekit import connect, LocationLocal, VehicleMode, Battery, SystemStatus, LocationGlobalRelative
from pymavlink import mavutil
from time import sleep


#connection_string = "/dev/ttyTHS1"
connection_string = '127.0.0.1:14540'
vehicle = connect(connection_string, baud=57600)

def send(param1,param2):
	vehicle._master.mav.command_long_send(
                1,  	# autopilot system id
                1,  	# autopilot component id
                35, 	# command id, 35 = VEHICLE_CMD_DO_KUSBEGI
                0,  	# confirmation
                param1, # param1
                param2,	# param2
		0,	# param3
		0,	# param4
		0,	# param5
		0,	# param6
		0  	# param7
        )



while(True):
	sleep(2)
	send(0,0)
	sleep(2)
	send(1,0)

