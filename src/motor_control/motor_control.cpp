#include "motor_control.h"
#define M_PI		3.14159265358979323846

/**
    Compute invertor voltages based on the rotor position relative to the stator.
*/


void MotorControl::computeVoltageVector(float rotorPosition, float vectorAmplitude, float* voltageVector)
{

    /// compute the electric angle of the rotor with respect to the initial position. Initial position theoretically represents q axis alignment with an A stator axis.
    float theta_elec_degrees = ((rotorPosition - _initialRotorPosition)*_polePairs + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90
	float theta = theta_elec_degrees*M_PI/180;//Pi/180; // translating into radians


    /// compute a projection of the voltage vector in the q axis onto the stator's axis
    float Va_1 = arm_cos_f32(theta);//cos(theta         );
	float Vb_1 = arm_cos_f32(theta - 2*M_PI/3);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	float Vc_1 = arm_cos_f32(theta + 2*M_PI/3);//cos(theta + 2.0943951023931954923084289221863);

	float Va = Va_1 * vectorAmplitude; // projection calculation of Vq into A phase
	float Vb = Vb_1 * vectorAmplitude; // projection calculation of Vq into B phase
	float Vc = Vc_1 * vectorAmplitude; // projection calculation of Vq into C phase


	/// Obtaining value for invertor, +50 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage. Vinv value should be 0-100%
	/// should also be taken into account that Vphase(max) = Vdc/2 (using sinusoidal commutation)
	float Vinv1 = Va/2 + 50;
	float Vinv2 = Vb/2 + 50;
	float Vinv3 = Vc/2 + 50;

	voltageVector[0] = Vinv1;
	voltageVector[1] = Vinv2;
	voltageVector[2] = Vinv3;
}



void PID_Controller::setCoefficients(float Kp, float Kd, float Ki)
{
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
}

void PID_Controller::setKp(float Kp)
{
    _Kp = Kp;
}


float PID_Controller::computeControlSignal(float desiredValue, float feedbackValue)
{
    /// if an error exceeds 180 degrees then the motor should rotate into the desired position in the direction of the least distance
    float error = desiredValue - feedbackValue;
    if(error > 180)
	{
		error = 360 - error;
		error = - error;
	}

	if(error < -180)
	{
		error = 360 + error;
	}

	float Vq = error*_Kp; /// P-control

	Vq = Vq > 100 ? 100 : (Vq < -100 ? -100 : Vq); ///  constraint check. Vq max amplitude is 100%

	return Vq;
}



/**
    This function sets the voltage vector along the stator axis A.
*/
void MotorControl::getInitVector(float* voltageVector)
{
    float Vq=50;  /// %

	float Va_1 = arm_cos_f32(0);//cos(theta         );
	float Vb_1 = arm_cos_f32(0 - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 ); //2*Pi/3
	float Vc_1 = arm_cos_f32(0 + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);

	float Va = Va_1 * Vq; // projection calculation of Vq into A phase
	float Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	float Vc = Vc_1 * Vq; // projection calculation of Vq into C phase


	/// Obtaining value for invertor, +50 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage. Vinv value should be 0-100%
	/// should also be taken into account that Vphase(max) = Vdc/2 (using sinusoidal commutation)
	float Vinv1 = Va/2 + 50;
	float Vinv2 = Vb/2 + 50;
	float Vinv3 = Vc/2 + 50;

	voltageVector[0] = Vinv1;
	voltageVector[1] = Vinv2;
	voltageVector[2] = Vinv3;

}

void MotorControl::setInitialMotorPositionValue(float initialRotorPosition)
{
    _initialRotorPosition = initialRotorPosition;
}

void MotorControl::setPolePairs(float polePairs)
{
    _polePairs = polePairs;
}
