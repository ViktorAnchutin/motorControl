#include "system.h"



/*
    Static variables.
*/

uint32_t System::sys_time_ms;
Platform System::_platform;
Encoder System::_encoder;
MotorControl System::_motorControl;
float System::_desiredRotorPosition;
PID_Controller System::_pidController;



/**
    Gets the encoder's relative position to the rotor.
    It is required to compute rotor's d-q axis position in the run time.
*/

void System::_initEncoderRelativePosition()
{
     /// get voltage vector to set the motor in the initial position
    float voltageVector[3] = {0};
    _motorControl.getInitVector(voltageVector);
    /// set motor in the initial position to measure the rotor's position
    _platform.setVoltageVector(voltageVector);
    blockingDelayMs(1000);
    _motorControl.setInitialMotorPositionValue(get_rotor_position());
}




void System::init()
{
    _platform.init();
    _encoder.init_interface();
    _motorControl.setPolePairs(11);
    _pidController.setKp(5);
    _initEncoderRelativePosition();
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


/**
    Start control loop timer.
*/

void System::start_control_loop()
{
    _platform.control_loop_timer_init();
}


void System::control_loop()
{
     float rotorPosition = get_rotor_position();
     float vectorAmplitude = _pidController.computeControlSignal(_desiredRotorPosition, rotorPosition); /// Use PID controller to compute a control signal
     float voltageVector[3] = {0};
     _motorControl.computeVoltageVector(rotorPosition, vectorAmplitude, voltageVector); /// compute the voltage vector based on the rotor's position
     _platform.setVoltageVector(voltageVector); /// set PWM
}






extern "C" void SysTick_Handler()
{
    System::sys_time_ms++;
}



/// control loop interrupt handler
extern "C" void TIM3_IRQHandler()
{

    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {

			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

            System::control_loop();
    }
}


void System::blockingDelayMs(uint32_t DelayMs)
{
    uint32_t startTime = sys_time_ms;
    while(sys_time_ms - startTime < DelayMs)
    {

    }
}


void System::setDesiredRotorPosition(float desPosition)
{
    _desiredRotorPosition = desPosition;
}



