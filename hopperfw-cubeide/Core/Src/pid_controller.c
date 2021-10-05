/*
 * pid_controller.c
 *
 *  Created on: May 31, 2021
 *      Author: Victo
 */
#include "pid_controller.h"
#include <math.h>
double PID_Process(PIDController* controller, float error){

	double deltaTime = osKernelGetTickCount() - controller->lastTick;

	deltaTime /= 1000;

	double out = 0.0;
	double pTerm = controller->kP * error;

	controller->errorSum += deltaTime * error;
	//controller->errorSum = clam(controller->errorSum, min, max);

	double iTerm = controller->kI * controller->errorSum;

	double dTerm = (error- controller->lastError) / deltaTime;

	dTerm *= controller->kD;

	controller->lastError = error;
	controller->lastTick = osKernelGetTickCount();
	out = pTerm + iTerm + dTerm;
	//output = clam(output, min ,max);

	return out;
}
