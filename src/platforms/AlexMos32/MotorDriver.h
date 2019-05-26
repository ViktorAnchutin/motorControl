#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER


#include "stm32f30x_gpio.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"

#define PWM_period  3600

/**

*/

class MotorDriver
{
public:

    MotorDriver(int index);
    ~MotorDriver(){};
    void init();
    void setVoltageVector(float* voltageVector);

private:

    uint8_t _driverNum;  /// driver index. There are 3 drivers on the board
};




#endif //MOTOR_DRIVER
