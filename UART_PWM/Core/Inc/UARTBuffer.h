#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define BUFFER_SIZE 100		// subject to change
#define MAX_MESSAGE_SIZE 200 // enough for 4 digits for all motor and frequency inputs

// Structure to represent a UART message
typedef struct {
    char message[MAX_MESSAGE_SIZE];
} UartMessage;

// Circular buffer structure
typedef struct {
    UartMessage messages[BUFFER_SIZE];
    int front;  // Index of the front of the buffer
    int rear;   // Index of the rear of the buffer
    int count;  // Number of messages in the buffer
} UartBuffer;

// Function to initialize the circular buffer
void initializeBuffer(UartBuffer *buffer);

// Function to add to the circular buffer
int addToBuffer(UartBuffer * buffer, const char * message, UART_HandleTypeDef *huart);

// Function to remove from the circular buffer
UartMessage removeFromBuffer(UartBuffer * buffer, UART_HandleTypeDef *huart);
