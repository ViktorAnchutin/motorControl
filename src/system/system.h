#pragma once


#include "stm32f30x_rcc.h"
#include "platform.h"

struct WaitData /// static WaitData wd1 = {sys.time_now(), 500}
{
    uint32_t start_time;
    uint32_t wait_time;
};


class System
{
public:

    static void init();

    static bool non_blocking_wait_ms(WaitData* wait_data);

    static void led_toggle();

    static void toggle_led_periodically(uint32_t period_ms);

    static uint32_t sys_time_ms;

private:

    System() = default;

    ~System() = default;

    static Platform _platform;


};
