#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	// coefficients of difference equation
	float c1;
	float c2;
	float c3;
	float c4;
	float c5;
	// control signals
	float setpoint; //reference
	float measured;
	float error; //maybe int16
	float output;
	// prev errors
	float errorN1;
	float errorN2;
	// prev outputs
	float outputN1;
	float outputN2;

}control_status_t;

extern volatile control_status_t *control;

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
);

float getC1(volatile control_status_t *control);
float getC2(volatile control_status_t *control);
float getC3(volatile control_status_t *control);
float getC4(volatile control_status_t *control);
float getC5(volatile control_status_t *control);
float getSetpoint(volatile control_status_t *control);
float getMeasured(volatile control_status_t *control);
float getError(volatile control_status_t *control);
float getOutput(volatile control_status_t *control);
float getErrorN1(volatile control_status_t *control);
float getErrorN2(volatile control_status_t *control);
float getOutputN1(volatile control_status_t *control);
float getOutputN2(volatile control_status_t *control);
void setC1(volatile control_status_t *control, float c1);
void setC2(volatile control_status_t *control, float c2);
void setC3(volatile control_status_t *control, float c3);
void setC4(volatile control_status_t *control, float c4);
void setC5(volatile control_status_t *control, float c5);
void setSetpoint(volatile control_status_t *control, float setpoint);
void setMeasured(volatile control_status_t *control, float measured);
void setError(volatile control_status_t *control, float error);
void setOutput(volatile control_status_t *control, float output);
void setErrorN1(volatile control_status_t *control, float errorN1);
void setErrorN2(volatile control_status_t *control, float errorN2);
void setOutputN1(volatile control_status_t *control, float outputN1);
void setOutputN2(volatile control_status_t *control, float outputN2);
void setKd(float value);
void setKi(float value);
void setKp(float value);
void computeError (volatile control_status_t *control);
uint32_t WritePWM(volatile control_status_t *control, float value);
float clamp(float value, float max, float min);
uint16_t map(uint16_t x, long in_min, long in_max, long out_min, long out_max);
void computePID (volatile control_status_t *control);
void updatePID (volatile control_status_t *control);
void updateCoefficients(void);

#endif /* INC_CONTROL_H_ */
