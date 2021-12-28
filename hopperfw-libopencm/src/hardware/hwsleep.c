#include "hardware/hwsleep.h"
volatile uint32_t system_millis;

/* Called when systick fires */
/*
void sys_tick_handler(void)
{
	system_millis++;
}

*/
/* sleep for delay milliseconds */
void hw_sleep_ms(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	//while (wake > system_millis);
}

/* Set up a timer to create 1mS ticks. */
void hw_sleep_init(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	//systick_set_reload(168000);
	//systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	//systick_counter_enable();
	/* this done last */
	//systick_interrupt_enable();
}
