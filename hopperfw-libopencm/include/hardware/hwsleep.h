#ifndef _VFC_HWSLEEP_H_
#define _VFC_HWSLEEP_H_

#include <stdbool.h>
#include <stdint.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

void hw_sleep_ms(uint32_t delay);
void hw_sleep_init(void);

#endif