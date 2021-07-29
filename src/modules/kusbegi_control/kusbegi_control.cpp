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
	vcmd.param5 = (double)param5;
	vcmd.param6 = (double)param6;
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
			//TODO: store the takeoff altitude
			break;
		case MCU_CMD_LAND:
			PX4_INFO("Land command received");
			break;
		default:
			break;
	}
}

void KusbegiControl::run_kusbegi(){

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
				_phase = FAIL_SAFE;
				_failsafe_reason = FS_CMD_TIMEOUT;
			}
		}
		break;
	default:
		break;
	}


	switch (_phase)
	{
	case IDLE:
		/* code */
		break;
	case TAKEOFF:
		//TODO: use stored take off altitude
		//now continue with default take off altitude

		//switch to takeoff mode and arm
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
		PX4_INFO("Kusbegi - Take off!!");
		_phase = IDLE;

		break;
	case TRANSITION:

		break;
	default:
		_phase = FAIL_SAFE;
		PX4_WARN("UNKNOWN STATE: Going failsafe");
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
	//TODO: start from argv state
	return 0;
}

int KusbegiControl::stop_mission(){

	return 0;
}

int KusbegiControl::status_mission(){

	return 0;
}

int KusbegiControl::test_func(){
	_phase = TAKEOFF;
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

