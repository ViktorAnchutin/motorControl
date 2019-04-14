#pragma once

#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include  "stm32f30x_spi.h"


class PlatformBase
{
public:

    PlatformBase(){};
    ~PlatformBase(){};

    virtual void spi_init()=0;
    //virtual void motor_driver_init()=0;
    //virtual void pwm_timer_init()=0;
    virtual void systick_timer_init()=0;
    //virtual void control_loop_timer_init()=0;
    //virtual void encoder_init()=0;
    virtual void led_init()=0;
    virtual void toggle_led()=0;

};
