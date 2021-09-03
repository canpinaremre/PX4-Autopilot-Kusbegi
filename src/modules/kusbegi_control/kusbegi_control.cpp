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
	vcmd.param7 = 490.5f;

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
	//TODO: check publisher
	switch ((int)_cmd_mission_s.param1)
	{
		case MCU_CMD_TAKEOFF:
			PX4_INFO("Take off command received");
			_phase = TAKEOFF;
			//TODO: store the takeoff altitude
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
	float min_dist_to_target = 3.0f;

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
// Required:
// ...TODO
void KusbegiControl::mission1()
{
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;

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
		// _wp is 2 update it to 3 and navigate from 3 to 6
		// WPs are line-2_top, line-2_bottom,  start, finish
		//               3            4          5	6
		_wp = 3;
		// 3 + 3 = 6 is the destination wp index
		navigate_MissionList(3);
		// _wp is now 6. But it doesn't matter.
		// Mission is done.
		// Go to land.
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
// Go to pool, line 1, detect red area while navigating to line 2,
// go to start. Go to pool, take water, line 1, red area, dump water,
// go to line 2, go to end.
// Required:
// ...TODO
void KusbegiControl::mission2()
{
	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;

	get_mcu_message();
	switch (_cmd_result)
	{
	case CMD_OK:
		handle_mcu_message();
		break;
	case CMD_ERROR:
		_cmd_err_cnt++;
		if(_cmd_err_cnt >= MAX_CMD_ERR_CNT){
			_phase = FAIL_SAFE;
			_failsafe_reason = FS_CMD_ERR;
		}
		break;
	case NO_CMD:
		if((hrt_absolute_time() - _last_cmd_time) > CMD_TIMEOUT){
			if(_phase == IDLE){
				//TODO: _phase = FAIL_SAFE;
				_failsafe_reason = FS_CMD_TIMEOUT;
			}
		}
		break;
	default:
		break;
	}


	switch (_stage)
	{
	case 0:
		/* Mission Not Started Yet */
		break;
	case 1:
		//Takeoff
		//_wait_stage = false; in mission start
		//first go to nav_takeoff state and wait 2 sec
		//_wait_stage = true;
		//check if we are loitering (takeoff done)
		//than restart _wait_stage = false; and next stage
		if((state_nav == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) && _wait_stage)
		{
			usleep(1_s);
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
		// WPs are line-1_bottom, line-1_top, start_redline
		//               0            1          2
		navigate_MissionList(2);
		// _wp is now 2

		_stage++;
		_wait_stage = false;

		break;


	case 3:
		// TODO: set yaw

		//Go with speed and detect red
		if(!_wait_stage){
			_wait_stage = true;

			startFlightTask();
			usleep(1_s);
			//TODO: adjust yaw

			usleep(200_ms);
			// TODO: NED to BODY fix
			sendSetpoint(kusbegi_target_s::KUSBEGI_DRV_TYPE_V,0.0f,5.0f,0.0f);

			//TODO: send MCU msg to detect red
			_timeout_time = hrt_absolute_time();
		}
		//get MCU red target msg
		//handle red target msg
		if(hrt_absolute_time() - _timeout_time > 10_s){
			//check distance or timeout if we are out of range
			_stage++;
			sendSetpoint(kusbegi_target_s::KUSBEGI_DRV_TYPE_V,0.0f,0.0f,0.0f);
			PX4_INFO("Timeout");
		}
		//CHECK WP 3 == stop_redline

		break;
	case 4:
		// update _wp to 4
		_wp = 4;
		// WPs are line-2_top, line-2_bottom,  start, pool
		//               4            5          6	7
		navigate_MissionList(3);
		// _wp is now 7

		_stage++;
		_wait_stage = false;

		break;
	case 5:
		//Take water
		_stage++;
		break;

	case 6:
		_wp = 0;
		// _wp is 0
		// WPs are line-1_bottom, line-1_top,
		//               0            1
		navigate_MissionList(1);

		_stage++;
		break;
	case 7:
		//Go to detected red coordinate

		_stage++;
		break;

	case 8:
		//dump water

		_stage++;
		break;

	case 9:
		// update _wp to 4
		_wp = 4;
		// WPs are line-2_top, line-2_bottom,  start
		//               4            5          6
		navigate_MissionList(2);
		// _wp is now 6
		_stage++;
		break;

	case 10:
		// go to end

		_stage++;
		break;

	case 11:
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
	case 12:
		//done
		//do nothing
		break;

		//Bak:
		// sendSetpoint(kusbegi_target_s::KUSBEGI_DRV_TYPE_V,5.0f,0.0f,0.0f);
		// usleep(10_s);
		// print_distance_global();
		// get_positionSetpoint();
		// sendSetpoint(kusbegi_target_s::KUSBEGI_DRV_TYPE_X,0.0f,5.0f,0.0f);

		//sendSetpoint(kusbegi_target_s::KUSBEGI_DRV_TYPE_X,0.0f,0.0f,0.0f);
		//check if reached

	default:
		break;
	}
}

void KusbegiControl::run_kusbegi(){

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;

	mission1();

	switch (_phase)
	{
	case IDLE:
		//TODO:reset _kusbegi_target_s
		// _kusbegi_target_s.drive_type = kusbegi_target_s::KUSBEGI_DRV_TYPE_IDLE;
		// _kusbegi_target_pub.publish(_kusbegi_target_s);
		break;
	case TAKEOFF:
		//TODO: use stored take off altitude
		//now continue with default take off altitude

		//switch to takeoff mode and arm
		// send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
		// send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
		// usleep(2_s);
		// PX4_INFO("Kusbegi - Take off!!");
		//_phase = TRANSITION;

		break;
	case TRANSITION:
		if(state_nav == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER)
		{
			_phase = IDLE;
			// _stage++;
		}

		break;
	case SPEED_IN_FRD_F:
		// _kusbegi_target_s.drive_type = kusbegi_target_s::KUSBEGI_DRV_TYPE_V;
		// _kusbegi_target_s.x = 5.0f;
		// _kusbegi_target_pub.publish(_kusbegi_target_s);

		break;
	default:
		// _phase = FAIL_SAFE;
		// PX4_WARN("UNKNOWN STATE: Going failsafe");
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
int KusbegiControl::start_mission(){
	//TODO: start from argv stage

	//stage 1 is takeoff
	_stage = 1;
	_wp = 0;
	_wait_stage = false;



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
	_phase = TAKEOFF;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	uint8_t state_nav = vehicle_status_sub.get().nav_state;
	PX4_INFO("Nav state: %d",state_nav);

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
	else if(!strcmp(verb, "start_mission")){
		return _object.load()->start_mission();
	}
	else if(!strcmp(verb, "stop_mission")){
		return _object.load()->stop_mission();
	}
	else if(!strcmp(verb, "status_mission")){
		return _object.load()->status_mission();
	}

	return print_usage("unknown command");
}

extern "C" __EXPORT int kusbegi_control_main(int argc, char *argv[])
{
	return KusbegiControl::main(argc, argv);
}

