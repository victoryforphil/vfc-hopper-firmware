/*
 * motor_driver.h
 *
 *  Created on: May 23, 2021
 *      Author: Victo
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_
#include <SEGGER_SYSVIEW.h>
#include <FreeRTOS.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

TIM_HandleTypeDef* _timer;
osSemaphoreId_t* _semaphore;



void MTR_Init(TIM_HandleTypeDef *timer, osSemaphoreId_t *semaphore);
void MTR_UpdateState(size_t values[4]);
void MTR_TaskUpdate(void *argument);


#endif /* INC_MOTOR_DRIVER_H_ */
