#pragma once
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#define KSB_MISSION_ITEM_COUNT 6

typedef struct
{
	uint8_t id;
	float lat;
	float lon;
}missionItem;

missionItem missionList[KSB_MISSION_ITEM_COUNT]{
	//SITL
	{0,47.397762f,8.546075f},
	{1,47.397843f,8.546050f},
	{2,47.397858f,8.545588f},
	{3,47.397824f,8.545149f},
	{4,47.397736f,8.545150f},
	{5,47.397740f,8.545641f}
};
