#pragma once





class PlatformBase
{
public:

    PlatformBase(){};
    ~PlatformBase(){};

    virtual void init()=0;
    virtual void systick_timer_init()=0;
    virtual void control_loop_timer_init()=0;
    virtual void led_init()=0;
    virtual void toggle_led()=0;
    virtual void setVoltageVector(float* voltageVector)=0;

};
