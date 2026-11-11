#include "serial.h"
#include <string.h>

static UART_HandleTypeDef *serialUart;
static uint8_t rxByte; //get a single character from the uart
static char rxBuffer[SERIAL_BUFFER_SIZE];//this is the big buffer (32 bytes)
static volatile uint16_t rxIndex = 0;
static volatile uint8_t commandReady = 0;

uint8_t txBuffer[4]; //Buffer for data TX

uint8_t txBuffer2[8];

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

void SerialSendPlot2(uint16_t v1, uint16_t v2)
{
    txBuffer[0] = (uint8_t)(v1 & 0xFF);
    txBuffer[1] = (uint8_t)((v1 >> 8) & 0xFF);

    txBuffer[2] = (uint8_t)(v2 & 0xFF);
    txBuffer[3] = (uint8_t)((v2 >> 8) & 0xFF);

    HAL_UART_Transmit_IT(serialUart, txBuffer, 4);
}

void SerialSendPlot4(uint16_t v1, uint16_t v2, uint16_t v3, uint16_t v4)
{

	txBuffer2[0] = (uint8_t)(v1 & 0xFF);
	txBuffer2[1] = (uint8_t)((v1 >> 8) & 0xFF);

	txBuffer2[2] = (uint8_t)(v2 & 0xFF);
	txBuffer2[3] = (uint8_t)((v2 >> 8) & 0xFF);

	txBuffer2[5] = (uint8_t)(v3 & 0xFF);
	txBuffer2[6] = (uint8_t)((v3 >> 8) & 0xFF);

	txBuffer2[7] = (uint8_t)(v4 & 0xFF);
	txBuffer2[8] = (uint8_t)((v4 >> 8) & 0xFF);

    HAL_UART_Transmit_IT(serialUart, txBuffer2, 8);
}

uint16_t decodeGcode(char *input) {
    if (strncmp(input, "C", 1) == 0) return CMD_CL;
    if (strncmp(input, "O", 1) == 0) return CMD_OL;
    if (strncmp(input, "P", 1) == 0) return CMD_KP;
    if (strncmp(input, "I", 1) == 0) return CMD_KI;
    if (strncmp(input, "D", 1) == 0) return CMD_KD;
    if (strncmp(input, "S", 1) == 0) return CMD_STOP;
    return CMD_NONE;
}

void ProcessCMD(char *input){
	int16_t cmd;
	int16_t value;

	cmd = decodeGcode(input);
	switch(cmd)
	{
		case CMD_CL:
			//cosed loop logic
			break;
		case CMD_OL:
			//open loop logic
			break;
		case CMD_KP:
			//change Ki
			break;
		case CMD_KI:
			//change Ki
			break;
		case CMD_KD:
			//change Kd
			break;
		case CMD_STOP:
			//emergency Stop
			break;
		case CMD_NONE:
		default:
			break;

	}
}

