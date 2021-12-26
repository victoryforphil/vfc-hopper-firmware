#ifndef VFC_HARDWARE_PWM_H
#define VFC_HARDWARE_PWM_H
#include <stdint.h>



int pwm_init(void);
int pwm_pinsetup(uint8_t chan);
int pwm_set(uint8_t channel, float value);
int pwm_enable();
#endif