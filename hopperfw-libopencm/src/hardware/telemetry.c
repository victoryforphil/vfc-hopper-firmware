
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>


int telem_init(void){
    // USART 1 
    // PA9 -> TX
    // PA10 -> RX
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO9 | GPIO10);

    // 

    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);


    dma_stream_reset(DMA1, )
    
}

void telem_log_info(char* message, uint32_t len){
    for(uint32_t i;i<len;i++){
        usart_send_blocking(USART1, 0x03);
    }
}
