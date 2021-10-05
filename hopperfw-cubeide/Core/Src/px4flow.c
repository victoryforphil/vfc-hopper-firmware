/*
 * px4flow.c
 *
 *  Created on: May 26, 2021
 *      Author: Victo
 */
#include "px4flow.h"
#define MAV_HEADER_SIZE 5;
PxParseState _currentState = PX_STATE_WAIT;
MavLinkV1Fame _currentFrame = {0};
OpticalFlowMessage _currentFlow = {0};
uint8_t _state = 0; // 0 = waiting for magic, 1 = waiting for header, 2 = waiting for data
uint8_t _currentBuffer[256];
uint8_t _currentIdx = 0;

extern osSemaphoreId_t semUART2DataHandle;
extern osMutexId_t mutexPX4Handle;
void pxReset(){

	PXNewData();
}
OpticalFlowMessage* pxGetLatest(){
	return &_currentFlow;
}
void pxUpdate(){






}

int pxParse(uint8_t* dataBuffer, size_t size , size_t* offset ){

	size_t startIdx=  size;
	for(size_t i =0; i<size; i++){
		uint8_t cur = dataBuffer[i] ;
		if(dataBuffer[i] == 0xFE){
			startIdx = i;
			break;
		}
	}

	if(startIdx >= (size - 6)){
		return 0;
	}

	if(_currentFrame.msgid == 101){
		size_t i = 0;
		i++;
	}

	memcpy(&_currentFrame, dataBuffer + startIdx, 6);
	if(_currentFrame.msgid == 100){
		memcpy(&_currentFlow, dataBuffer + (startIdx+ 6 ), _currentFrame.len);
	}


	return 64;
}

