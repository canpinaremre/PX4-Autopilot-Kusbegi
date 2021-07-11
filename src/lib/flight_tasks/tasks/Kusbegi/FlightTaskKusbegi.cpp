#include "FlightTaskKusbegi.hpp"

using namespace matrix;

void FlightTaskKusbegi::_bodyToNedFrame(float xBody,float yBody,float yawBody)
{
	//translate body frame to NED frame and publish position setpoints.
	//_position_setpoint(0) = _origin_x + ( xBody * cosf(yawBody) ) - ( yBody * sinf(yawBody) );
	//_position_setpoint(1) = _origin_y + ( xBody * sinf(yawBody) ) + ( yBody * cosf(yawBody) );
}


bool FlightTaskKusbegi::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{

	bool ret = FlightTask::activate(last_setpoint);


	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2));

	if(!ret){
		return ret;
	}


	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_position_setpoint(2) = _position(2);
	_yaw_setpoint = _yaw;

	_kusbegi_mission_s.timestamp = hrt_absolute_time();
	_kusbegi_mission_s.kusbegi_state = 2;
	_kusbegi_mission_pub.publish(_kusbegi_mission_s);

	return ret;

}


bool FlightTaskKusbegi::update()
{



	return true;
}
