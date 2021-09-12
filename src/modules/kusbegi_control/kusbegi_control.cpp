/**
 * @file kusbegi_control.cpp
 * @author canpinaremre
 * @brief
 * @version 0.1
 * @date 2021-07-12
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "kusbegi_control.h"
using namespace time_literals;
static bool send_vehicle_command(uint16_t cmd, float param1 = NAN, float param2 = NAN, float param3 = NAN,
				 float param4 = NAN, float param5 = NAN, float param6 = NAN, float param7 = NAN)
{
	vehicle_command_s vcmd{};

	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	vcmd.command = cmd;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	vcmd.timestamp = hrt_absolute_time();

	uORB::PublicationQueued<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};

	return vcmd_pub.publish(vcmd);
}

void KusbegiControl::do_reposition(){
	vehicle_command_s vcmd{};

	vcmd.param1 = -1.0f;
	vcmd.param2 = 1.0f;
	vcmd.param3 = 0.0f;
	vcmd.param4 = NAN;
	vcmd.param5 = _target_lat;
	vcmd.param6 = _target_lon;
	vcmd.param7 = _mission_alt;

	vcmd.command = 192;//reposition


	vcmd.source_system = 255;
	vcmd.target_system = 1;
	vcmd.source_component = 190;
	vcmd.target_component = 1;

	vcmd.timestamp = hrt_absolute_time();

	uORB::PublicationQueued<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};

	vcmd_pub.publish(vcmd);

}

float KusbegiControl::get_distance_global()
{
	_global_pos_sub.updated();
	_global_pos_sub.copy(&_global_pos_s);
	//TODO: make c casts static_cast
	return get_distance_to_next_waypoint((double)_global_pos_s.lat,(double)_global_pos_s.lon,
						(double)_target_lat,(double)_target_lon);
}
void KusbegiControl::print_distance_global()
{
	PX4_INFO("Distance to global target: %f meters", static_cast<double>(get_distance_global()));
}

KusbegiControl::KusbegiControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

KusbegiControl::~KusbegiControl()
{
	//already cleared in Run()
	//ScheduleClear();
}

int KusbegiControl::task_spawn(int argc, char *argv[])
{
	KusbegiControl *obj = new KusbegiControl();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

/**
 * @brief Init function of the module
 *
 */
void KusbegiControl::start()
{
	_phase = IDLE;


	ScheduleOnInterval(500_ms); // 2 Hz
}

/**
 * @brief Main function
 * This function will run in every Schedule Interval
 * which is specified in start()
 *
 */
void KusbegiControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	run_kusbegi();
}
void KusbegiControl::handle_mcu_message(){

	switch ((int)_cmd_mission_s.param1)
	{
		case MCU_CMD_TAKEOFF:
			PX4_INFO("Take off command received");
			_phase = TAKEOFF;
			break;
		case MCU_CMD_LAND:
			PX4_INFO("Land command received");
			break;
		default:
			break;
	}
}

void KusbegiControl::get_positionSetpoint(){
	// float trajPos[3];
	// _local_pos_sub.updated();
	// _local_pos_sub.copy(&_local_pos_s);
	// trajPos[0] = _local_pos_s.x;
	// trajPos[1] = _local_pos_s.y;
	// trajPos[2] = _local_pos_s.z;
	// PX4_INFO("SP X: %f",(double)trajPos[0]);
	// PX4_INFO("SP Y: %f",(double)trajPos[1]);
	// PX4_INFO("SP Z: %f",(double)trajPos[2]);

	_global_pos_sub.updated();
	_global_pos_sub.copy(&_global_pos_s);

	PX4_INFO("Lat: %f",(double)_global_pos_s.lat);
	PX4_INFO("Lon: %f",(double)_global_pos_s.lon);
}

// Go to waypoints in missionList
// starting from index _wp to _wp + targetWp
// updates _wp to last navigated waypoint index
void KusbegiControl::navigate_MissionList(int targetWp)
{

	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
					     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
	for(int i = 0; i <= targetWp; i++)
	{
		PX4_INFO("Mission Next WP: %d",_wp+i);
		_target_lat = missionList[_wp+i].lat;
		_target_lon = missionList[_wp+i].lon;
		do_reposition();

		while(get_distance_global() > min_dist_to_target)
		{
			//wait
			usleep(1_s);
		}

	}
	_wp += targetWp;
}

void KusbegiControl::do_circle()
{
	//full stop to get nice _position
	usleep(2_s);
	startFlightTask();
	//give some time to task

	float radius = 10;
	while(true)
	{
		_control_to_task.timestamp = hrt_absolute_time();
		_control_to_task.mission = kusbegi_control_to_task_s::MISSON_DO_CIRCLE;
		_control_to_task.param1 = radius;
		_control_to_task.param2 = _circle_yaw;
		_kusbegi_control_to_task.publish(_control_to_task);

		// Give some time to take uorb message and send back
		usleep(100_ms);
		if (_kusbegi_task_to_control.update(&_task_to_control))
		{

			if(_task_to_control.status == kusbegi_task_to_control_s::CIRCLE_STARTED)
			{
				PX4_INFO("break");
				//Started circle, stop sending
				break;
			}

		}
	}


	//Wait for flight task do finish circle
	while(true)
	{

		if (_kusbegi_task_to_control.update(&_task_to_control))
		{

			if(_task_to_control.status == kusbegi_task_to_control_s::CIRCLE_DONE)
			{
				PX4_INFO("break");
				//Started circle, stop sending
				break;
			}

		}
		usleep(100_ms);
	}
	PX4_INFO("Circle done!");


}

// First mission. No need to get mcu messages.
void KusbegiControl::mission1()
{
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;
	min_dist_to_target = 4.0f;
	switch (_stage)
	{
	case 0:
		/* Mission Not Started Yet */
		break;
	case 1:
		// Takeoff
		// _wait_stage = false; in mission start
		// first go to nav_takeoff state and wait 2 sec
		// _wait_stage = true;
		// check if we are loitering (takeoff done)
		// than restart _wait_stage = false; and next stage
		if((state_nav == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) && _wait_stage)
		{
			usleep(1_s);
			_global_pos_sub.updated();
			_global_pos_sub.copy(&_global_pos_s);
			_mission_alt = _global_pos_s.alt;
			_wait_stage = false;
			_stage++;
			break;
		}

		if(!_wait_stage){
			_wait_stage = true;
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
			PX4_INFO("Kusbegi - Take off!!");
			usleep(2_s);
		}

		break;

	case 2:
		// _wp is 0
		// WPs are line-1_bottom, line-1_top, center
		//               0            1          2
		navigate_MissionList(2);
		// _wp is now 2

		_stage++;
		_wait_stage = false;

		break;
	case 3:
		// Do circle around the pole
		PX4_INFO("Circle!");
		do_circle();

		_stage++;
		break;
	case 4:

		_wp = 3;
		if(first_run)
		{
			// First run just go tp line-2 top and line-2bottom
			// than we will go to stage 2 in case 5
			navigate_MissionList(1);
		}
		else
		{
			// _wp is 2 update it to 3 and navigate from 3 to 6
			// WPs are line-2_top, line-2_bottom,  start, finish
			//               3            4          5	6
			// 3 + 3 = 6 is the destination wp index
			navigate_MissionList(3);
			// _wp is now 6. But it doesn't matter.
			// Mission is done.
			// Go to land.
		}
		_stage++;


		break;
	case 5:


		_stage++;

		if(first_run)
		{
			first_run = false;
			_stage = 2;
			_wp = 0;
		}
		else
		{
			PX4_INFO("Landing");
		}

		break;
	case 6:
		//land

		if(state_nav == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
		{
			_stage++;
		}
		else{
			usleep(1_s);
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
					     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

		}

		break;
	case 7:
		//Do nothing
		break;

	default:
		break;
	}
}

// Mission 2
void KusbegiControl::mission2()
{
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;

	get_mcu_message();


	switch (_stage)
	{
	case 0:
		/* Mission Not Started Yet */
		break;
	case 1:
		PX4_INFO("Stage 1");
		//Takeoff
		//_wait_stage = false; in mission start
		//first go to nav_takeoff state and wait 2 sec
		//_wait_stage = true;
		//check if we are loitering (takeoff done)
		//than restart _wait_stage = false; and next stage
		if((state_nav == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) && _wait_stage)
		{
			usleep(1_s);
			_global_pos_sub.updated();
			_global_pos_sub.copy(&_global_pos_s);
			_mission_alt = _global_pos_s.alt;
			_wait_stage = false;
			_stage++;
			break;
		}

		if(!_wait_stage){
			_wait_stage = true;
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
			PX4_INFO("Kusbegi - Take off!!");
			usleep(2_s);
		}

		break;

	case 2:
		PX4_INFO("Stage 2");
		// _wp is 0
		// WPs are line-1, red-1,
		//          0        1
		navigate_MissionList(1);
		// _wp is now 1

		_stage++;
		_wait_stage = false;

		break;


	case 3:
		PX4_INFO("Stage 3");
		// wp 2 is red-2
		// set speed very slow and jerk small
		// and reposition

		_param_volatile = param_handle(px4::params::MPC_XY_CRUISE);
		// save param value
		param_get(_param_volatile,&_xy_cruise_float);

		// set param as:
		_param_float = 2.0f;
		param_set(_param_volatile,&_param_float);

		_param_volatile = param_handle(px4::params::MPC_JERK_AUTO);
		// save param value
		param_get(_param_volatile,&_jerk_float);

		// set param as:
		_param_float = 2.0f;
		param_set(_param_volatile,&_param_float);


		_wp = 2;
		_target_lat = missionList[_wp].lat;
		_target_lon = missionList[_wp].lon;
		do_reposition();

		while(get_distance_global() > min_dist_to_target)
		{
			// detect red TODO:
			_global_pos_sub.updated();
			_global_pos_sub.copy(&_global_pos_s);
			if(1)//red bigger TODO:
			{
				_target_red_area = _global_pos_s;
			}
			usleep(1_s);
		}


		// set old parameters
		_param_volatile = param_handle(px4::params::MPC_XY_CRUISE);
		param_set(_param_volatile,&_xy_cruise_float);

		_param_volatile = param_handle(px4::params::MPC_JERK_AUTO);
		param_set(_param_volatile,&_jerk_float);


		_stage++;

		break;
	case 4:
		PX4_INFO("Stage 4");
		min_dist_to_target = 2.0f;
		// update _wp to 3
		_wp = 3;
		// WPs are line-2, pool,
		//          3       4
		navigate_MissionList(1);
		// _wp is now 4

		_stage++;
		_wait_stage = false;

		break;
	case 5:
		PX4_INFO("Stage 5");
		//Take water TODO:


		startFlightTask();
		usleep(1_s);

		while(true)
		{
			_control_to_task.timestamp = hrt_absolute_time();
			_control_to_task.mission = kusbegi_control_to_task_s::MISSON_DESCEND;
			_control_to_task.param1 = -1.5f;
			_kusbegi_control_to_task.publish(_control_to_task);

			// Give some time to take uorb message and send back
			usleep(100_ms);
			if (_kusbegi_task_to_control.update(&_task_to_control))
			{

				if(_task_to_control.status == kusbegi_task_to_control_s::DESCENDING)
				{
					PX4_INFO("Descending");
					//Started descending
					break;
				}

			}
		}

		while(true)
		{

			if (_kusbegi_task_to_control.update(&_task_to_control))
			{

				if(_task_to_control.status == kusbegi_task_to_control_s::DESCENDING_DONE)
				{
					PX4_INFO("DESCENDING_DONE");
					//Started circle, stop sending
					break;
				}

			}
			usleep(100_ms);
		}

		send_message_to_mcu(MCU_TAKE_WATER,0.0f);
		PX4_INFO("Taking water for 10 secs");
		usleep(10_s);
		send_message_to_mcu(MCU_STATE_IDLE,0.0f);

		// ascend from pool
		// same coordinate with mission altitude
		_wp = 4;
		navigate_MissionList(0);
		//since we are not checking altitude in navigate
		usleep(3_s);

		_stage++;
		break;
	case 6:
		PX4_INFO("Stage 6");
		// pass line1
		_wp = 0;
		navigate_MissionList(0);
		_stage++;

		break;

	case 7:
		PX4_INFO("Stage 7");
		// go to red area
		_target_lat = _target_red_area.lat;
		_target_lon = _target_red_area.lon;
		do_reposition();
		min_dist_to_target = 2.0f;
		while(get_distance_global() > min_dist_to_target)
		{
			usleep(1_s);
		}

		_stage++;

		break;
	case 8:
		PX4_INFO("Stage 8");
		// TODO:red land

		startFlightTask();
		usleep(1_s);

		while(true)
		{
			_control_to_task.timestamp = hrt_absolute_time();
			_control_to_task.mission = kusbegi_control_to_task_s::MISSON_RED;
			_kusbegi_control_to_task.publish(_control_to_task);

			// Give some time to take uorb message and send back
			usleep(100_ms);
			if (_kusbegi_task_to_control.update(&_task_to_control))
			{

				if(_task_to_control.status == kusbegi_task_to_control_s::RED_LANDING)
				{
					PX4_INFO("RED_LANDING");
					//Started descending
					break;
				}

			}
		}

		while(true)
		{

			if (_kusbegi_task_to_control.update(&_task_to_control))
			{

				// get mcu message
				get_mcu_message();
				//get red detection and send to flight task
				_control_to_task.timestamp = hrt_absolute_time();
				_control_to_task.mission = kusbegi_control_to_task_s::MISSON_RED;
				_control_to_task.param1 = _cmd_mission_s.param1;
				_control_to_task.param2 = _cmd_mission_s.param2;
				_kusbegi_control_to_task.publish(_control_to_task);

				// If too close to ground send signal from task
				if(_task_to_control.status == kusbegi_task_to_control_s::RED_LANDED)
				{
					PX4_INFO("RED_LANDED");
					//Started circle, stop sending
					break;
				}

			}
			usleep(100_ms);
		}


		_stage++;
		break;

	case 9:
		PX4_INFO("Stage 9");
		send_message_to_mcu(MCU_DUMP_WATER,0.0f);
		PX4_INFO("Dumping water for 10 secs");
		usleep(10_s);
		send_message_to_mcu(MCU_STATE_IDLE,0.0f);

		min_dist_to_target = 4.0f;

		do_reposition();
		usleep(3_s);

		_stage++;
		break;

	case 10:
		PX4_INFO("Stage 10");
		// update _wp to 3 and 5 =  line2 and finish
		_wp = 3;
		navigate_MissionList(0);

		_wp = 5;
		navigate_MissionList(0);

		_stage++;
		break;

	case 11:
		PX4_INFO("Stage 11");
		// go to end

		_stage++;
		break;

	case 12:
		PX4_INFO("Stage 12");
		//land

		if(state_nav == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
		{
			_stage++;
		}
		else{
			usleep(1_s);
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
					     PX4_CUSTOM_SUB_MODE_AUTO_LAND);

		}

		break;
	case 13:
		//done
		//do nothing
		break;

	default:
		break;
	}
}

void KusbegiControl::run_kusbegi(){

	// uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	// uint8_t state_nav = vehicle_status_sub.get().nav_state;

	switch(_active_mission)
	{
	case 0:
		// None
		break;
	case 1:
		mission1();
		break;
	case 2:
		mission2();
		break;
	default:
		break;
	}


}

/**
 * @brief Get the mcu message object
 *
 * @return true on success
 * @return false else
 */
bool KusbegiControl::get_mcu_message(){

	if (_kusbegi_mission_sub.updated()) {
		/* got command */
		if (!_kusbegi_mission_sub.copy(&_cmd_mission_s)) {
			_cmd_result = CMD_ERROR;
			return false;
		}
		_cmd_result =CMD_OK;
		return true;
	}
	_last_cmd_time = hrt_absolute_time();
	_cmd_result = NO_CMD;
	return false;
}

/**
 * @brief Send message to mcu
 *
 * @return true on success
 * @return false else
 */
bool KusbegiControl::send_message_to_mcu(mcuSetState state,float fparam){
	float fstate = (float) state;
	bool ret =false;

	updateParams();

	_required_state.set(fstate);
	_param_custom.set(fparam);

	ret = (_required_state.commit() && _param_custom.commit());

	updateParams();

return ret;
}


int KusbegiControl::print_status(){

	PX4_INFO("Status test: %d",mytest);
	PX4_INFO("Cmd result: %d",_cmd_result);
	return 0;
}

int KusbegiControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Kusbegi control module
)DESCR_STR");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}
int KusbegiControl::start_mission1(){
	//TODO: start from argv stage

	//stage 1 is takeoff
	_stage = 1;
	_wp = 0;
	_wait_stage = false;

	param_t param = param_handle(px4::params::MIS_TAKEOFF_ALT);
	float takeoff_alt = 10.0f;
	param_set(param,&takeoff_alt);
	_active_mission = 1;


	return 0;
}

int KusbegiControl::start_mission2(){
	//TODO: start from argv stage

	//stage 1 is takeoff
	_stage = 1;
	_wp = 0;
	_wait_stage = false;

	param_t param = param_handle(px4::params::MIS_TAKEOFF_ALT);
	float takeoff_alt = 10.0f;
	param_set(param,&takeoff_alt);
	_active_mission = 2;

	return 0;
}

int KusbegiControl::startFlightTask(){
	PX4_INFO("Entering FlightTask Kusbegi");
	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_KUSBEGI);

	return 0;
}
int KusbegiControl::takeOff(){
	_phase = TAKEOFF;
	return 0;

}
int KusbegiControl::sendSetpoint(uint8_t drive_type, float x, float y, float z){


	_kusbegi_target_s.drive_type = drive_type;
	_kusbegi_target_s.x = x;
	_kusbegi_target_s.y = y;
	_kusbegi_target_s.z = z;
	_kusbegi_target_pub.publish(_kusbegi_target_s);
	return 0;

}

int KusbegiControl::stop_mission(){
	_phase = IDLE;
	return 0;
}

int KusbegiControl::status_mission(){
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;
	PX4_INFO("Nav state: %d",state_nav);
	get_positionSetpoint();

	return 0;
}

int KusbegiControl::test_func(){
	send_message_to_mcu(MCU_DUMP_WATER,0.0f);
	usleep(2_s);
	send_message_to_mcu(MCU_TAKE_WATER,0.0f);
	usleep(5_s);
	send_message_to_mcu(MCU_STATE_IDLE,0.0f);

	return 0;
}

int KusbegiControl::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!is_running()){
		PX4_INFO("Not running!");
		return PX4_ERROR;
	}


	if (!strcmp(verb, "test")) {
		return _object.load()->test_func();
	}
	else if(!strcmp(verb, "start_mission1")){
		return _object.load()->start_mission1();
	}
	else if(!strcmp(verb, "stop_mission")){
		return _object.load()->stop_mission();
	}
	else if(!strcmp(verb, "status_mission")){
		return _object.load()->status_mission();
	}
	else if(!strcmp(verb, "start_mission2")){
		return _object.load()->start_mission2();
	}

	return print_usage("unknown command");
}

extern "C" __EXPORT int kusbegi_control_main(int argc, char *argv[])
{
	return KusbegiControl::main(argc, argv);
}

