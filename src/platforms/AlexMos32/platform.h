#pragma once

#include "platform_base.h"



class Platform: public PlatformBase
{
public:
    Platform(){};
    ~Platform(){};

    void spi_init() override;
    void toggle_led() override;
    void systick_timer_init() override;
    void led_init() override;

};
