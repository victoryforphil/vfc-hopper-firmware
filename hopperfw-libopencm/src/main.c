#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include "hardware/px4flow.h"
#include "hardware/telemetry.h"

static void clock_setup(void){

    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    // Enable GPIO Clocks A,B,C
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_DMA1); // for I2C DMA
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART1);
}
int main(void){
    clock_setup();
    
    px4flow_init();
    telem_init();


    while (1) {

       
		
	}


    return 0;
}