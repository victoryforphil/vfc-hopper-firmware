#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include "hardware/px4flow.h"
#include "hardware/telemetry.h"
#include "hardware/pwm.h"
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "systems/motors.h"
#include "hardware/hwsleep.h"

static void clock_setup(void){

    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    // Enable GPIO Clocks A,B,C
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_DMA1); // for I2C DMA
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_TIM1);
}
int main(void){
    clock_setup();
    hw_sleep_init();
    hw_sleep_ms(3000);

    vfc_motor_init(5);
    
    //px4flow_init();
    //telem_init();

    float tick = 0.0;
    float delta = 0.015;
    while (1) {
        hw_sleep_ms(100);
        vfc_motor_set_speed_all(tick);
        vfc_motor_tick();
        tick += delta;
        if(tick > 1.0){
            delta = -delta;
        }else if(tick < 0){
            delta = -delta;
        }
		
	}


    return 0;
}