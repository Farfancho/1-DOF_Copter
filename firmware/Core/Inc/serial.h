#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

#define SERIAL_BUFFER_SIZE 32

void serialInit(UART_HandleTypeDef *huart);
uint8_t serialAvailable(void); //if the data is not over recieving returns zero
void serialReadLine(char *destination, uint16_t maxLength);

#endif /* INC_SERIAL_H_ */
