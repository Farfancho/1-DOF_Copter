#include "control.h"

#define OUTPUTMAX 999
#define OUTPUTMIN 0

volatile control_status_t control_data; //creacion de la variable
volatile control_status_t *control = &control_data; //puntero a la variable

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
	control->outputN1 = outputN2;
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
// GETTERS AND SETTERS END

// BASIC LOGIC BEGIN


uint16_t clamp(float value, float max, float min)
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

// BASIC LOGIC END

// CLOSED LOOP LOGIC BEGIN

void computeError (volatile control_status_t *control)
{
	control->error = control->setpoint - control->measured;
}

// CLOSED LOOP LOGIC END

// OPEN LOOP LOGIC BEGIN
uint32_t WritePWM(volatile control_status_t *control, float value)
{
	value = clamp(value, OUTPUTMAX, OUTPUTMIN);
	uint32_t pwmValue = 90000 + (value * 90);
	return pwmValue;
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);
}
// OPEN LOOP LOGIC END


