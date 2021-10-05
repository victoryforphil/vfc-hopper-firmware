#include "hardware/bno055.h"

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>


uint8_t i2cBuffer[256] = {0};

int bno055_init(){
    // Initialize I2C
    i2c_reset(I2C1);

    // Set GPIO Pins (PB7 -> SDA, PB6 -> SCL)
    // AF = Alternate Function
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    // Set the AF Mode (reference the AF table in datasheet) to AF4 (I2C1/2/3)
    gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);

    //Setup DMA

    nvic_enable_irq(NVIC_I2C1_EV_IRQ); // Enable EVent interrupt


    i2c_peripheral_disable(I2C1);
    i2c_enable_analog_filter(I2C1);
    i2c_set_digital_filter(I2C1, 0);

    i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);

    i2c_set_7bit_addr_mode(I2C1);
    
    //I2C STREAM 0, CHAN 1 Rx

    nvic_set_priority(NVIC_DMA1_STREAM0_IRQ, 0 );
    nvic_enable_irq(NVIC_DMA1_STREAM0_IRQ);

    dma_stream_reset(DMA1, DMA_STREAM0);
    dma_set_priority(DMA1, DMA_STREAM0, DMA_SxCR_PL_LOW);
    dma_set_memory_size(DMA1, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM0);
    dma_set_transfer_mode(DMA1, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA1, DMA1_STREAM0, (uint32_t) &I2C1_DR );
    dma_set_memory_address(DMA1, DMA_STREAM0, (uint32_t) i2cBuffer);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM0);
    dma_channel_select(DMA1, DMA1_STREAM0, DMA_SxCR_CHSEL_1);
    dma_enable_stream(DMA1, DMA_STREAM0);
    
    i2c_enable_dma(I2C1);
	i2c_peripheral_enable(I2C1);
    
}
