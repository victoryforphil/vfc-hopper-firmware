#include "systems/motors.h"
//#include "../utils/logging.h"
#include "hardware/hwsleep.h"
#include "hardware/pwm.h"
static VFC_MotorState _motor_state = {0};

void vfc_motor_init(uint8_t wait)
{

    // log_info("Motors", "init", "Init Motors, Setting Power to 0.01");
    pwm_init();
    pwm_pinsetup(1);
    pwm_pinsetup(2);
    pwm_pinsetup(3);
    pwm_pinsetup(4);
    pwm_enable();
    vfc_motor_set_speed_all(0.05);

    // log_info("Motors", "init", "Waiting for ESC's (fixed time) ");

    hw_sleep_ms(wait * 1000);

    // log_success("Motors", "init", "ESC's are ready");
}

void vfc_motor_task()
{
    // log_info("Motors", "task", "Starting RTOS Task");
    while (1)
    {
        vfc_motor_tick();
        hw_sleep_ms(10);
    }
}

void vfc_motor_tick()
{
    pwm_set(1, _motor_state.m_speed[0]);
    pwm_set(2, _motor_state.m_speed[1]);
    pwm_set(3, _motor_state.m_speed[2]);
    pwm_set(4, _motor_state.m_speed[3]);

}

void vfc_motor_set_speed(int motor, float speed)
{
    _motor_state.m_speed[motor - 1] = speed;
}
void vfc_motor_set_speed_all(float speed)
{
    for (int i = 0; i < 4; i++)
    {
        _motor_state.m_speed[i] = speed;
    }
}

void vfc_motor_get_state(VFC_MotorState *outState)
{
    *outState = _motor_state;
}
void vfc_motor_set_state(VFC_MotorState *state)
{
    _motor_state = *state;
}

void vfc_motor_kill()
{

    vfc_motor_set_speed_all(0.0);
}
