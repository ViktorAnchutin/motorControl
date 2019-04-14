#pragma once

#include "platform_base.h"




class Platform: public PlatformBase
{
public:
    Platform(){};
    ~Platform(){};
    void init() override;
    void toggle_led() override;
    void systick_timer_init() override;
    void led_init() override;
    void control_loop_timer_init() override;
};
