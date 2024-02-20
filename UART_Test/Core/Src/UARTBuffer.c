#include <stdio.h>
#include <string.h>
#include "UARTBuffer.h"
#include "main.h"
#include "stm32f4xx_hal.h"

// Function to initialize the circular buffer
void initializeBuffer(UartBuffer *buffer) {
	buffer->front = 0;
	buffer->rear = -1;
	buffer->count = 0;
}

// Function to add to the circular buffer
void addToBuffer(UartBuffer * buffer, const char * message, UART_HandleTypeDef *huart) {
	if (buffer->count < BUFFER_SIZE) {
        buffer->rear = (buffer->rear + 1) % BUFFER_SIZE;
        strncpy(buffer->messages[buffer->rear].message, message, MAX_MESSAGE_SIZE - 1);
        buffer->messages[buffer->rear].message[MAX_MESSAGE_SIZE - 1] = '\0'; // Ensure null-termination
        buffer->count++;
	} else {
		HAL_UART_TRANSMIT(huart, (const *) "Buffer Full\n", 12, 100);
	}
}

// Function to remove from the circular buffer
UartMessage removeFromBuffer(UartBuffer * buffer, UART_HandleTypeDef *huart) {
	UartMessage underflowMessage;
	underflowMessage.message[0] = '\0';

	if (buffer->count > 0) {
        UartMessage dequeuedMessage = buffer->messages[buffer->front];
        buffer->front = (buffer->front + 1) % BUFFER_SIZE;
        buffer->count--;
        return dequeuedMessage;
	} else {
		HAL_UART_TRANSMIT(huart, (const *) "Buffer Underflow\n", 17, 100);
		return underflowMessage;
	}
}
