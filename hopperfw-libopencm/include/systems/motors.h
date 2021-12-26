#ifndef _VFC_MOTORS_H_
#define _VFC_MOTORS_H_

#include <stdbool.h>
#include <stdint.h>
typedef struct VFC_MotorState_t{
    float m_speed[4];
} VFC_MotorState;

void vfc_motor_init(uint8_t wait);
void vfc_motor_task();
void vfc_motor_tick();
void vfc_motor_set_speed(int motor, float speed);
void vfc_motor_set_speed_all(float speed);
void vfc_motor_get_state(VFC_MotorState* outState);
void vfc_motor_set_state(VFC_MotorState* state);
void vfc_motor_kill();

#endif