#include "control.h"

#define OUTPUTMAX 999
#define OUTPUTMIN 0
#define CONTROL_TS 0.02f

volatile control_status_t control_data; //creacion de la variable
volatile control_status_t *control = &control_data; //puntero a la variable

static float Kp = 4.0878f;
static float Ki = 5.72f;
static float Kd = 0.0f;

extern volatile bool flag;

void controlInit
(
		volatile control_status_t *control,

		float c1,
		float c2,
		float c3,
		float c4,
		float c5,

		float setpoint,
		float measured,
		float error,
		float output,

		float errorN1,
		float errorN2,
		float outputN1,
		float outputN2
){
	control->c1 = c1;
	control->c2 = c2;
	control->c3 = c3;
	control->c4 = c4;
	control->c5 = c5;
	control->setpoint = setpoint;
	control->measured = measured;
	control->error = error;
	control->output = output;
	control->errorN1 = errorN1;
	control->errorN2 = errorN2;
	control->outputN1 = outputN1;
	control->outputN2 = outputN2;

}

// GETTERS AND SETTERS BEGIN

float getC1(volatile control_status_t *control)
{
    return control->c1;
}

float getC2(volatile control_status_t *control)
{
    return control->c2;
}

float getC3(volatile control_status_t *control)
{
    return control->c3;
}

float getC4(volatile control_status_t *control)
{
    return control->c4;
}

float getC5(volatile control_status_t *control)
{
    return control->c5;
}

float getSetpoint(volatile control_status_t *control)
{
    return control->setpoint;
}

float getMeasured(volatile control_status_t *control)
{
    return control->measured;
}

float getError(volatile control_status_t *control)
{
    return control->error;
}

float getOutput(volatile control_status_t *control)
{
    return control->output;
}

float getErrorN1(volatile control_status_t *control)
{
    return control->errorN1;
}

float getErrorN2(volatile control_status_t *control)
{
    return control->errorN2;
}

float getOutputN1(volatile control_status_t *control)
{
    return control->outputN1;
}

float getOutputN2(volatile control_status_t *control)
{
    return control->outputN2;
}

void setC1(volatile control_status_t *control, float c1)
{
    control->c1 = c1;
}

void setC2(volatile control_status_t *control, float c2)
{
    control->c2 = c2;
}

void setC3(volatile control_status_t *control, float c3)
{
    control->c3 = c3;
}

void setC4(volatile control_status_t *control, float c4)
{
    control->c4 = c4;
}

void setC5(volatile control_status_t *control, float c5)
{
    control->c5 = c5;
}

void setSetpoint(volatile control_status_t *control, float setpoint)
{
    control->setpoint = setpoint;
}

void setMeasured(volatile control_status_t *control, float measured)
{
    control->measured = measured;
}

void setError(volatile control_status_t *control, float error)
{
    control->error = error;
}

void setOutput(volatile control_status_t *control, float output)
{
    control->output = output;
}

void setErrorN1(volatile control_status_t *control, float errorN1)
{
    control->errorN1 = errorN1;
}

void setErrorN2(volatile control_status_t *control, float errorN2)
{
    control->errorN2 = errorN2;
}

void setOutputN1(volatile control_status_t *control, float outputN1)
{
    control->outputN1 = outputN1;
}

void setOutputN2(volatile control_status_t *control, float outputN2)
{
    control->outputN2 = outputN2;
}

void setKp(float value)
{
    Kp = value;
}

void setKi(float value)
{
    Ki = value;
}

void setKd(float value)
{
    Kd = value;
}
// GETTERS AND SETTERS END

// BASIC LOGIC BEGIN

float clamp(float value, float max, float min)
{
	if (value > max)
	{
		value = max;
	}
	else if (value < min)
	{
		value = min;
	}
	return value;
}

uint32_t WritePWM (volatile control_status_t *control, float value)
{
	value = clamp(value, OUTPUTMAX, OUTPUTMIN);
	uint32_t pwmValue = 90000 + (value * 90);
	return pwmValue;
}

uint16_t map(uint16_t x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min + 1)
           / (in_max - in_min + 1)
           + out_min;
}

// BASIC LOGIC END

// CLOSED LOOP LOGIC BEGIN

void computeError (volatile control_status_t *control)
{
	control->error = control->setpoint - control->measured;
}


void computePID (volatile control_status_t *control)
{
	control->output =
			control->c1 * control->outputN1 +
			control->c2 * control->outputN2 +
			control->c3 * control->error +
			control->c4 * control->errorN1 +
			control->c5 * control->errorN2;
}

void updatePID (volatile control_status_t *control)
{
	//update errors
	control->errorN2 = control->errorN1;
	control->errorN1 = control->error;

	//update outputs
	control->outputN2 = control->outputN1;
	control->outputN1 = control->output;
}

void updateCoefficients(void)
{
    bool previousFlag = flag;
    flag = false;

    float integral = Ki * CONTROL_TS / 2.0f;
    float derivative = Kd / CONTROL_TS;

    setC1(control, 1.0f);
    setC2(control, 0.0f);

    setC3(control, Kp + integral + derivative);
    setC4(control, -Kp + integral - 2.0f * derivative);
    setC5(control, derivative);

    flag = previousFlag;
}
// CLOSED LOOP LOGIC END


