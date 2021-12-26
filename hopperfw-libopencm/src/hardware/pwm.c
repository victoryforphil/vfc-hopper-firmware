#include "hardware/pwm.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>



int pwm_init(void){

    gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO13 | GPIO14);
	gpio_set_af(GPIOE, GPIO_AF1, GPIO9 | GPIO11 | GPIO13 | GPIO14);
	gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,  GPIO9 | GPIO11 | GPIO13 | GPIO14);



    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 7);
    timer_set_repetition_counter(TIM1, 0);
    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);

    /* Period (32kHz). */
	/* TIM1 uses a clock from APB2, see reg RCC_APB2ENR in STM32F407 RM */
	/* APB2 PREESC was set to 2, so APB2 timer clocks is */
	/* 2 x rcc_apb2_frequency, see clock tree in STM32F407 RM */
    timer_set_period(TIM1, rcc_apb2_frequency / 1500);

    /* Configure break and deadtime. */
	timer_set_deadtime(TIM1, 10);
	timer_set_enabled_off_state_in_idle_mode(TIM1);
	timer_set_enabled_off_state_in_run_mode(TIM1);
	timer_disable_break(TIM1);
	timer_set_break_polarity_high(TIM1);
	timer_disable_break_automatic_output(TIM1);
	timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);


	/* ARR reload enable. */
	timer_enable_preload(TIM1);

	/* Enable outputs in the break subsystem. */
	timer_enable_break_main_output(TIM1);



}

int pwm_pinsetup(uint8_t chan){
    enum tim_oc_id chanId , chanNId;

    if(chan == 1){
        chanId = TIM_OC1;
        chanNId = TIM_OC1N;
    }
    else if(chan == 2){
        chanId = TIM_OC2;
        chanNId = TIM_OC2N;
    }
    else if(chan == 3){
        chanId = TIM_OC3;
        chanNId = TIM_OC3N;
    }
    else if(chan == 4){
        chanId = TIM_OC4;
        chanNId = TIM_OC4;
    }

    /* -- OC3 configuration -- */
	/* Disable outputs. */
	timer_disable_oc_output(TIM1, chanId);

	/* Configure global mode of line 3. */
	timer_disable_oc_clear(TIM1, chanId);
	timer_enable_oc_preload(TIM1, chanId);
	timer_set_oc_slow_mode(TIM1, chanId);
	timer_set_oc_mode(TIM1, chanId, TIM_OCM_PWM1);

	/* Configure OC3. */
	timer_set_oc_polarity_high(TIM1, chanId);
	timer_set_oc_idle_state_set(TIM1, chanId);

	/* Set the capture compare value for OC3. 50% duty */
	

	/* Reenable outputs. */
	timer_enable_oc_output(TIM1, chanId);
    pwm_set(chan, 0.01);
}
int pwm_set(uint8_t channel, float value){
    enum tim_oc_id chanId , chanNId;

    if(channel == 1){
        chanId = TIM_OC1;
        chanNId = TIM_OC1N;
    }
    else if(channel == 2){
        chanId = TIM_OC2;
        chanNId = TIM_OC2N;
    }
    else if(channel == 3){
        chanId = TIM_OC3;
        chanNId = TIM_OC3N;
    }
    else if(channel == 4){
        chanId = TIM_OC4;
        chanNId = TIM_OC4;
    }
    timer_set_oc_value(TIM1, chanId, rcc_apb2_frequency * ((value * 1.3) + 0.65 ) /3450);
}

int pwm_enable(){
    timer_enable_counter(TIM1);
}