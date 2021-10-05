#include "hardware/px4flow.h"

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>

uint8_t uart2Buffer[PX4_BUFFER_SIZE] = {0};

int px4flow_init(){

    // UART 2
    // GPIO: PA2 -> TX, PA3 -> RX

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD,GPIO_OSPEED_100MHZ, GPIO2 | GPIO3 );
    

    // RX Channel: Stream 5, Channel 4

    nvic_set_priority(NVIC_DMA1_STREAM5_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_STREAM5_IRQ);
    nvic_enable_irq(NVIC_USART2_IRQ);
    nvic_set_priority(NVIC_USART2_IRQ, 0);

    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_enable(USART2);
    


    dma_stream_reset(DMA1, DMA_STREAM5);
    dma_set_priority(DMA1, DMA_STREAM5, DMA_SxCR_PL_LOW);
    dma_set_memory_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1,DMA_STREAM5);
    dma_set_transfer_mode(DMA1, DMA_STREAM5, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA1, DMA_STREAM5, (uint32_t)&USART2_DR);
    dma_set_memory_address(DMA1, DMA_STREAM5,  (uint32_t) uart2Buffer);
    dma_set_number_of_data(DMA1, DMA_STREAM5, PX4_BUFFER_SIZE);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM5);
    dma_enable_half_transfer_interrupt(DMA1, DMA_STREAM5);
    dma_enable_fifo_error_interrupt(DMA1, DMA_STREAM5);
    dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_4);
    dma_enable_stream(DMA1, DMA_STREAM5);
    
    
    usart_enable_rx_dma(USART2);
    usart_enable_rx_interrupt(USART2);
   
    usart_enable_error_interrupt(USART2);
    return 0;
}

void dma1_stream5_isr(void)
{
	
}

void usart2_isr(void){
    if(usart_get_flag(USART2, USART_FLAG_IDLE)){
        uint8_t i =0;
        
        i++;
    }
}