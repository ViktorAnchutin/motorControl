#include "system.h"


uint32_t System::sys_time_ms;
Platform System::_platform;
Encoder System::_encoder;

void System::init()
{
    _platform.init();
    _encoder.init_interface();

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
    if( non_blocking_wait_ms(&wd) )
    {
      led_toggle();
    }
}



float System::get_rotor_position()
{
    return _encoder.get_angle();
}


void System::start_control_loop()
{
    _platform.control_loop_timer_init();
}

float debug_angle;
void System::control_loop()
{
     debug_angle = get_rotor_position();
}






extern "C" void SysTick_Handler()
{
    System::sys_time_ms++;
}


extern "C" void TIM3_IRQHandler()
{

    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {

			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

            System::control_loop();
    }
}



