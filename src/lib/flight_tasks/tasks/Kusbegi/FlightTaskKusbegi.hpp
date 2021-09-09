#pragma once

#include "FlightTask.hpp"

#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/kusbegi_mission.h>
#include <uORB/topics/kusbegi_target.h>
#include <uORB/topics/kusbegi_control_to_task.h>
#include <uORB/topics/kusbegi_task_to_control.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

class FlightTaskKusbegi : public FlightTask
{

public:
	FlightTaskKusbegi() = default;
	virtual ~FlightTaskKusbegi() = default;

	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool update() override;


private:
	enum flightPhase {
		IDLE = 0,
		TAKEOFF = 1,
		TO_WP = 2,
		LAND = 3,
		FLIGHT_TASK = 4,
		FAIL_SAFE = 5,
		TRANSITION =6,
		DRV_TYPE_V = 7,
		DRV_TYPE_X = 8,
		CIRCLE_MODE = 9
	};

	flightPhase _phase{flightPhase::IDLE};
	kusbegi_mission_s	_kusbegi_mission_s{};
	kusbegi_target_s	_kusbegi_target_s{};


	uORB::Publication<kusbegi_mission_s>			_kusbegi_mission_pub{ORB_ID(kusbegi_mission)};
	uORB::Publication<kusbegi_target_s>			_kusbegi_target_pub{ORB_ID(kusbegi_target)};

	uORB::Subscription					_kusbegi_target_sub{ORB_ID(kusbegi_target)};
	uORB::Subscription					_kusbegi_mission_sub{ORB_ID(kusbegi_mission)};

	// Flight task and module communication
	kusbegi_task_to_control_s _task_to_control{};
	kusbegi_control_to_task_s _control_to_task{};
	uORB::Subscription					_kusbegi_control_to_task{ORB_ID(kusbegi_control_to_task)};
	uORB::Publication<kusbegi_task_to_control_s>		_kusbegi_task_to_control{ORB_ID(kusbegi_task_to_control)};

	void _bodyToNedFrame(float xBody,float yBody,float yawBody);
	void _calculatePositionFromSpeed();
	void _calculateSpeedFromTargetSpeed();
	void _gotoOffset();
	void _sendPosition();
	void _resetSpeed();
	void _resetSPs();
	void _do_circle();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise /**< cruise speed for circle approach */
	)

	float _local[3];
	float _local_yaw;
	float _ksb_v[3];
	float _target_v[3];
	float _takeoff_pos[3];
	float _ksb_a;
	float _max_target_speed;
	float _offsetApplied[3];
	float _offsetSpeed[3];

	bool _shouldDrive = false;
	float _radius_of_circle;
	float _v = 0;
	matrix::Vector2f _center;


protected:

};
