#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL


#define __FPU_PRESENT             1

#include "arm_math.h"
#include "math.h"



class PID_Controller
{
public:
    PID_Controller(){}
    ~PID_Controller(){}
    float computeControlSignal(float desiredValue, float feedbackValue);
    void setKp(float Kp);
    void setKd(float Kd);
    void setKi(float Kp);
    void setCoefficients(float Kp, float Kd, float Ki);

private:
    float _Kp;
    float _Kd;
    float _Ki;
    float _integral; /// error integral
    float _difference; /// error difference

};

class MotorControl
{
public:
    MotorControl(){}
    ~MotorControl(){}
    void computeVoltageVector(float rotorPosition, float vectorAmplitude, float* voltageVector);
    void initRotorPosition();
    void getInitVector(float* voltageVector);
    void setInitialMotorPositionValue(float initialRotorPosition);
    void setPolePairs(float polePairs);

private:
    float _initialRotorPosition;
    float _polePairs;



};








#endif// MOTOR_CONTROL
