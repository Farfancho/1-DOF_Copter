#include "serial.h"
#include <string.h>

static UART_HandleTypeDef *serialUart;
static uint8_t rxByte; //get a single character from the uart
static char rxBuffer[SERIAL_BUFFER_SIZE];//this is the big buffer (32 bytes)
static volatile uint16_t rxIndex = 0;
static volatile uint8_t commandReady = 0;

void serialInit(UART_HandleTypeDef *huart){
	serialUart = huart;
	rxIndex = 0;
	commandReady = 0;

	HAL_UART_Receive_IT(serialUart, &rxByte, 1);
}

uint8_t serialAvailable(void){
	return commandReady;
}

void serialReadLine(char *destination, uint16_t maxLength){
	if(!serialAvailable())
		return;

	strncpy(destination, rxBuffer, maxLength - 1);

	destination[maxLength - 1] = '\0';

	commandReady = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == serialUart){
		if (!commandReady){
			if (rxByte != '\r'){
				if(rxByte == '\n'){

					rxBuffer[rxIndex] = '\0';
					rxIndex = 0;
					commandReady = 1;

				}else{
					if(rxIndex < SERIAL_BUFFER_SIZE - 1){

						rxBuffer[rxIndex] = rxByte;
						rxIndex++;

					}else{
						rxIndex = 0;
					}
				}
			}
		}
		HAL_UART_Receive_IT(serialUart, &rxByte, 1);
	}
}
