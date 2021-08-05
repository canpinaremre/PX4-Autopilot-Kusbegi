#pragma once
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#define KSB_MISSION_ITEM_COUNT 3

typedef struct
{
	uint8_t id;
	float lat;
	float lon;
}missionItem;

missionItem missionList[KSB_MISSION_ITEM_COUNT]{
	{0,5,5},
	{1,4,4},
	{2,3,3}
};
