#pragma once

#include "FlightTask.hpp"

#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/kusbegi_mission.h>
#include <uORB/topics/kusbegi_target.h>

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
	kusbegi_mission_s	_kusbegi_mission_s{};
	kusbegi_target_s	_kusbegi_target_s{};


	uORB::Publication<kusbegi_mission_s>			_kusbegi_mission_pub{ORB_ID(kusbegi_mission)};
	uORB::Publication<kusbegi_target_s>			_kusbegi_target_pub{ORB_ID(kusbegi_target)};

	uORB::Subscription					_kusbegi_target_sub{ORB_ID(kusbegi_target)};
	uORB::Subscription					_kusbegi_mission_sub{ORB_ID(kusbegi_mission)};
	void _bodyToNedFrame(float xBody,float yBody,float yawBody);
protected:

};
