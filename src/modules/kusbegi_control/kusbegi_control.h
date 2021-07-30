#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <commander/px4_custom_mode.h>

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/kusbegi_mission.h>
#include <uORB/topics/kusbegi_target.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>



using namespace time_literals;

#define MAX_CMD_ERR_CNT 3
#define CMD_TIMEOUT 15_s
class KusbegiControl : public ModuleBase<KusbegiControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	KusbegiControl();
	~KusbegiControl() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void start();
	int print_status() override;
	void run_kusbegi();

private:
	int sendSetpoint(uint8_t drive_type, float x, float y, float z);
	int startFlightTask();
	int takeOff();
	int start_mission();
	int stop_mission();
	int status_mission();
	int test_func();
	/** Do a compute and schedule the next cycle. */
	void Run() override;

	kusbegi_mission_s	_kusbegi_mission_s{};
	kusbegi_target_s	_kusbegi_target_s{};

	kusbegi_mission_s	_cmd_mission_s{};


	uORB::Publication<kusbegi_mission_s>			_kusbegi_mission_pub{ORB_ID(kusbegi_mission)};
	uORB::Publication<kusbegi_target_s>			_kusbegi_target_pub{ORB_ID(kusbegi_target)};

	uORB::Subscription					_kusbegi_target_sub{ORB_ID(kusbegi_target)};
	uORB::Subscription					_kusbegi_mission_sub{ORB_ID(kusbegi_mission)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>)
		_required_state,	/**< Required state from mcu */
		(ParamFloat<px4::params::NAV_FW_ALTL_RAD>)
		_param_custom	/**< Custom param for future use*/
	)

	enum flightPhase {
		IDLE = 0,
		TAKEOFF = 1,
		TO_WP = 2,
		LAND = 3,
		FLIGHT_TASK = 4,
		FAIL_SAFE = 5,
		TRANSITION =6,
		SPEED_IN_FRD_F = 7
	};

	enum commandResult{
		CMD_OK,
		CMD_ERROR,
		NO_CMD
	};

	enum mcuCommand{
		MCU_CMD_TAKEOFF = 0,
		MCU_CMD_LAND = 1
	};

	enum mcuSetState{
		MCU_STATE_IDLE,
		MCU_STATE_RED_VALUE,
		MCU_STATE_RED_POSITION,
		MCU_STATE_POOL,

		MCU_STATE_WP1,
		MCU_STATE_WP2,
		MCU_STATE_WP3,
		MCU_STATE_WP4,
		MCU_STATE_WP5,
		MCU_STATE_WP6,
		MCU_STATE_WP7,
		MCU_STATE_WP8,
		MCU_STATE_WP9,
		MCU_STATE_WP10,
		MCU_STATE_WP11,
		MCU_STATE_WP12,
		MCU_STATE_WP13,
		MCU_STATE_WP14,
	};

	enum failsafeReason{
		FS_CMD_ERR,
		FS_CMD_TIMEOUT
	};
	failsafeReason _failsafe_reason{};
	commandResult _cmd_result{NO_CMD};
	flightPhase _phase{flightPhase::IDLE};
	int mytest{0};
	uint64_t _last_cmd_time{hrt_absolute_time()};
	uint32_t _cmd_err_cnt{0};
	bool _mission_active{false};

	bool get_mcu_message();
	bool send_message_to_mcu(mcuSetState state,float fparam);
	void handle_mcu_message();
};
