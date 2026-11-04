#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

#define SERIAL_BUFFER_SIZE 32

typedef enum
{
	CMD_NONE,
	CMD_OL,
	CMD_CL
};

void serialInit(UART_HandleTypeDef *huart);
uint8_t serialAvailable(void); //if the data is not over receiving returns zero
void serialReadLine(char *destination, uint16_t maxLength);
void SerialSendPlot2(uint16_t v1, uint16_t v2);



#endif /* INC_SERIAL_H_ */
