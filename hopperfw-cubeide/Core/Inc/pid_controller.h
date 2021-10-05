/*
 * pid_controller.h
 *
 *  Created on: May 31, 2021
 *      Author: Victo
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_
#include "main.h"
#include "cmsis_os.h"

typedef struct PidController_S{
	double kP, kI, kD;
	double lastError, errorSum;

	uint64_t lastTick;

} PIDController;

double PID_Process(PIDController* controller, float error);

#endif /* INC_PID_CONTROLLER_H_ */
