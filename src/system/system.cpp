#include "system.h"

uint32_t System::sys_time_ms;
Platform System::_platform;

void System::init()
{
    _platform.systick_timer_init();
    _platform.led_init();

}


bool System::non_blocking_wait_ms(WaitData* wait_data)
{
    if(sys_time_ms > wait_data->start_time + wait_data->wait_time)
    {
        wait_data->start_time = sys_time_ms;
        return 1; /// waiting time expired
    }
    else
    {
        return 0; /// still waiting
    }
}



void System::led_toggle()
{
    _platform.toggle_led();
}


void System::toggle_led_periodically(uint32_t period_ms)
{
    static WaitData wd{sys_time_ms, period_ms};
    if( non_blocking_wait_ms(&wd) ) led_toggle();
}







extern "C" void SysTick_Handler()
{
    System::sys_time_ms++;
}



