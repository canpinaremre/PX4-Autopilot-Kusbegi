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

KusbegiControl::KusbegiControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

KusbegiControl::~KusbegiControl()
{
	ScheduleClear();
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
	if(get_mcu_message()){
		mytest = (int)_cmd_mission_s.param2;
		print_status();
	}


	switch (_phase)
	{
	case IDLE:
		/* code */
		break;
	case TAKEOFF:
		/* code */
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
	_cmd_result = NO_CMD;
	return false;
}

/**
 * @brief Send message to mcu
 *
 * @return true on success
 * @return false else
 */
bool KusbegiControl::send_message_to_mcu(){
	//Orbit gibi kullanılmayan bir modül mesajları üzerinden
	//yada kullanılmayan bir parametre üzerinden veri basılabilir.

return true;
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

extern "C" __EXPORT int kusbegi_control_main(int argc, char *argv[])
{
	return KusbegiControl::main(argc, argv);
}

