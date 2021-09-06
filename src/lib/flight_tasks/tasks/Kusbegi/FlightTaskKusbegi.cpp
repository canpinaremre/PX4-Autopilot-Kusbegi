#include "FlightTaskKusbegi.hpp"

using namespace matrix;
using namespace time_literals;


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

	for(int i = 0; i < 3; i++)
	{
		_position_setpoint(i) = _position(i);
		_local[i] = _position(i);
		_takeoff_pos[i] = _position(i);
		_target_v[i] = 0.0f;
		_ksb_v[i] = 0.0f;

	}
	//TODO: get from params
	_ksb_a = 0.1f;
	_max_target_speed = 6.0f;
	_yaw_setpoint = _yaw;
	_local_yaw = _yaw;

	_resetSpeed();

	// _kusbegi_mission_s.timestamp = hrt_absolute_time();
	// _kusbegi_mission_s.kusbegi_state = 2;
	// _kusbegi_mission_pub.publish(_kusbegi_mission_s);

	_center = Vector2f(_position);
	_task_to_control.status = kusbegi_task_to_control_s::NONE;
	PX4_INFO("Flight Task Activated!");

	return ret;

}

void FlightTaskKusbegi::_do_circle()
{
	_v = 5.0f;
	float _r = _radius_of_circle;
	_shouldDrive = false;
	_phase = CIRCLE_MODE;

	const Vector2f center_to_position = Vector2f(_position) - _center;

	// xy velocity to go around in a circle
	Vector2f velocity_xy(-center_to_position(1), center_to_position(0));
	velocity_xy = velocity_xy.unit_or_zero();
	velocity_xy *= _v;

	// xy velocity adjustment to stay on the radius distance
	velocity_xy += (_r - center_to_position.norm()) * center_to_position.unit_or_zero();

	_position_setpoint(0) = _position_setpoint(1) = NAN;
	_velocity_setpoint.xy() = velocity_xy;
	_acceleration_setpoint.xy() = -center_to_position.unit_or_zero() * _v * _v / _r;

	_yaw_setpoint = wrap_pi(atan2f(center_to_position(1), center_to_position(0)) + (sign(_v) * M_PI_F / 2.f));
	_yawspeed_setpoint = _v / _r;

}



bool FlightTaskKusbegi::update()
{
	_shouldDrive = true;
	static bool shouldCircle=false;
	static Vector2f first_pos;
	static uint32_t circleTime;


	if (_kusbegi_control_to_task.update(&_control_to_task))
	{
		if(_control_to_task.mission == kusbegi_control_to_task_s::MISSON_DO_CIRCLE)
		{
			_radius_of_circle = _control_to_task.param1;
			float circle_yaw = _control_to_task.param2;
			shouldCircle = true;
			_center = Vector2f(_position);
			_center(0) += _radius_of_circle * cosf(circle_yaw);
			_center(1) += _radius_of_circle * sinf(circle_yaw);
			first_pos = Vector2f(_position);
			circleTime = hrt_absolute_time();

			_task_to_control.status = kusbegi_task_to_control_s::CIRCLE_STARTED;
		}
	}
	if(shouldCircle)
	{
		_do_circle();
		if(
			(hrt_absolute_time() - circleTime) > 2_s &&
			fabsf(_position(0) - first_pos(0)) < 1.5f &&
			fabsf(_position(1) - first_pos(1)) < 1.5f

		)
		{
			_task_to_control.status = kusbegi_task_to_control_s::CIRCLE_DONE;
			shouldCircle = false;
			_position_setpoint = _position;

		}
		else
		{
			_task_to_control.status = kusbegi_task_to_control_s::CIRCLE_STARTED;

		}

	}

	_task_to_control.timestamp = hrt_absolute_time();
	_kusbegi_task_to_control.publish(_task_to_control);






	if(_kusbegi_target_sub.updated() && _shouldDrive){
		_kusbegi_target_sub.update(&_kusbegi_target_s);

		switch (_kusbegi_target_s.drive_type)
		{
		case kusbegi_target_s::KUSBEGI_DRV_TYPE_IDLE:
			_phase = IDLE;
			break;
		case kusbegi_target_s::KUSBEGI_DRV_TYPE_V:
			_phase = DRV_TYPE_V;
			break;
		case kusbegi_target_s::KUSBEGI_DRV_TYPE_X:
			_resetSPs();
			_offsetApplied[0] = _local[0] + _kusbegi_target_s.x;
			_offsetApplied[1] = _local[1] + _kusbegi_target_s.y;
			_offsetApplied[2] = _local[2] + _kusbegi_target_s.z;
			_offsetSpeed[0] = 0.0f;
			_offsetSpeed[1] = 0.0f;
			_offsetSpeed[2] = 0.0f;
			_phase = DRV_TYPE_X;
			break;
		default:
			break;
		}
	}



	switch (_phase)
	{
	case IDLE:
		/* do nothing */
		_resetSpeed();
		break;
	case DRV_TYPE_V:
			_target_v[0] = _kusbegi_target_s.x;
			_target_v[1] = _kusbegi_target_s.y;
			_target_v[2] = _kusbegi_target_s.z;
		break;
	case DRV_TYPE_X:
			_gotoOffset();
		break;
	case CIRCLE_MODE:
		return true;
		break;
	default:
		break;
	}

	_calculateSpeedFromTargetSpeed();
	_calculatePositionFromSpeed();
	_sendPosition();
	_yaw_setpoint = _local_yaw;

	return true;
}

void FlightTaskKusbegi::_resetSPs(){

	for(int i = 0; i < 3; i++)
	{
		_position_setpoint(i) = _position(i);
		_local[i] = _position(i);
		_takeoff_pos[i] = _position(i);
		_target_v[i] = 0.0f;
		_ksb_v[i] = 0.0f;

	}

}

void FlightTaskKusbegi::_gotoOffset()
{


	for(int i = 0; i < 3; i++)
	{
		if(_offsetSpeed[i] < 2.0f){
			_offsetSpeed[i] += 0.06f;
		}

		if(fabsf(_offsetApplied[i] - _local[i]) < 0.05f){
			_local[i] = _offsetApplied[i];
			continue;
		}
		if(fabsf(_offsetApplied[i] - _local[i]) < 2.05f){
			_offsetSpeed[i] -= 0.12f;
			if(_offsetSpeed[i] < 0.0f){
				_offsetSpeed[i] = 0.08;
			}
		}

		if(_offsetApplied[i] > _local[i])
		{
			_local[i] += _offsetSpeed[i] * _deltatime;
		}
		else if(_offsetApplied[i] < _local[i])
		{
			_local[i] -= _offsetSpeed[i] * _deltatime;
		}
	}

}

void FlightTaskKusbegi::_calculateSpeedFromTargetSpeed()
{
	for(int i = 0; i < 3; i++)
	{
		if(fabsf(_target_v[i] - _ksb_v[i]) < (_ksb_a * 2.0f))
		{
			continue;
		}

		if(_target_v[i] > _ksb_v[i]){
			_ksb_v[i] += _ksb_a;
		}
		else if(_target_v[i] < _ksb_v[i]){
			_ksb_v[i] -= _ksb_a;
		}

	}
}
void FlightTaskKusbegi::_calculatePositionFromSpeed()
{
	for(int i = 0; i < 3; i++)
	{
		if(fabsf(_ksb_v[i]) < (_ksb_a * 0.9f)){
			continue;
		}
		_local[i] += _ksb_v[i] * _deltatime;
	}
}
void FlightTaskKusbegi::_sendPosition()
{
	for(int i = 0; i < 3; i++)
	{
		_position_setpoint(i) = _local[i];
	}
}
void FlightTaskKusbegi::_resetSpeed(){

	for(int i = 0; i < 3; i++)
	{
		_target_v[i] = 0;
	}
}

