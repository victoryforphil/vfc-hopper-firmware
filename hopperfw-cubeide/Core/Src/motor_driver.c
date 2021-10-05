/*
 * motor_driver.c
 *
 *  Created on: May 23, 2021
 *      Author: Victo
 */

#include "motor_driver.h"

size_t _mtrValues[4] = {1000,1000,1000,1000};

void MTR_Init(TIM_HandleTypeDef *timer, osSemaphoreId_t *semaphore){


	_timer = timer;
	size_t initValues[4] = {1005,1005,1005,1005};
	MTR_UpdateState(initValues);

	_semaphore = semaphore;
}

void MTR_UpdateState(size_t values[4]){
	_mtrValues[0] = values[0];
	_mtrValues[1] = values[1];
	_mtrValues[2] = values[2];
	_mtrValues[3] = values[3];
	//osSemaphoreRelease(_semaphore);
	//osSemaphoreWait(semaphore_id, millisec)
}


void MTR_TaskUpdate(void *argument){
	//osStatus_t status = osSemaphoreAcquire(_semaphore, 100);

	//if(status != 0){
	//	return;
	//}

	for(;;){
		//SEGGER_SYSVIEW_Printf("Updating PWM Values to %f", _values[0]);
		_timer->Instance->CCR1 = _mtrValues[0];
		_timer->Instance->CCR2 = _mtrValues[1];
		_timer->Instance->CCR3 = _mtrValues[2];
		_timer->Instance->CCR4 = _mtrValues[3];
		osDelay(10);
	}


}
