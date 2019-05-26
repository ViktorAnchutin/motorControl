#ifndef BOARD
#define BOARD



#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"

#include "platform_base.h"
#include "MotorDriver.h"





class Platform: public PlatformBase
{
public:
    Platform() : _motorDriver_1(1) {}
    ~Platform(){}
    void init() override;
    void toggle_led() override;
    void systick_timer_init() override;
    void led_init() override;
    void control_loop_timer_init() override;
    void setVoltageVector(float* voltageVector) override;

private:
    MotorDriver _motorDriver_1; /// drv8313 motor driver with integrated transistors
};


#endif //BOARD
