/*
 * px4flow.h
 *
 *  Created on: May 26, 2021
 *      Author: Victo
 */

#ifndef INC_PX4FLOW_H_
#define INC_PX4FLOW_H_
#include "main.h"
#include "cmsis_os.h"
typedef struct MavLinkV1Fame_s{
	uint8_t magic;
	uint8_t len;
	uint8_t seq;
	uint8_t sysid;
	uint8_t compid;
	uint8_t msgid;

}MavLinkV1Fame;
typedef struct OpticalFlowMessage_s{
	uint64_t time_usec;
	float flow_comp_m_x;
	float flow_comp_m_y;
	float ground_distance;
	int16_t flow_x;
	int16_t flow_y;
	uint8_t sensor_id;
	uint8_t quality;
}OpticalFlowMessage;

typedef enum EPxParseState {PX_STATE_WAIT = 0, PX_STATE_HEADER = 1, PX_STATE_DATA=2} PxParseState;
int pxParse(uint8_t* dataBuffer, size_t size, size_t* offset);
OpticalFlowMessage* pxGetLatest();
#endif /* INC_PX4FLOW_H_ */
