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

float _circle_yaw = -1.57f; // YAW POINTING TO CENTER OF CIRCLE

missionItem missionList[KSB_MISSION_ITEM_COUNT]{
	//SITL
	{0,40.2305352f,29.0093886f},
	{1,40.2305253f,29.0091090f},
	{2,40.2301438f,29.0090469f},
	{3,40.2297208f,29.0092540f},
	{4,40.2297208f,29.0094559f},
	{5,40.2301997f,29.0097918f}
};
