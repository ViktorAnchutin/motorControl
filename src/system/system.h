#ifndef SYSTEM_H
#define SYSTEM_H


#include "motor_control.h"
#include "platform.h"
#include "encoder.h"


/**
    Structure is used for non blocking delay function.
*/

struct WaitData
{
    uint32_t start_time;
    uint32_t wait_time;
};





class System
{
public:

    static void init();

    static bool non_blocking_wait_ms(WaitData* wait_data);

    static void blockingDelayMs(uint32_t DelayMs);

    static void led_toggle();

    static void toggle_led_periodically(uint32_t period_ms);

    static uint32_t sys_time_ms;

    static float get_rotor_position();

    static void start_control_loop();

    static void control_loop();

    static void setDesiredRotorPosition();

    static void setDesiredRotorPosition(float desPosition);


private:

    System() = default;

    ~System() = default;

    static Platform _platform;

    static Encoder _encoder;

    static MotorControl _motorControl;

    static PID_Controller _pidController;

    static float _desiredRotorPosition;

    static void _initEncoderRelativePosition();

};



#endif //SYSTEM_H
