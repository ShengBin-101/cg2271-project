#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "cmsis_os2.h"

// UART configuration constants
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22 // UART2 TX on Port E pin 22 (not used in code but included for reference)
#define UART_RX_PORTE23 23 // UART2 RX on Port E pin 23
#define UART2_INT_PRIO 128 // Interrupt priority

// Size of the UART transmit/receive buffer queue
#define Q_SIZE 128

// Structure to define a circular queue
typedef struct {
    unsigned char DATA[Q_SIZE]; // Storage buffer
    unsigned int HEAD; // Read pointer
    unsigned int TAIL; // Write pointer
    unsigned int SIZE; // Number of elements in queue
} Q_T;

// Queue instances declared as extern so they can be used in other files
extern Q_T tx_q, rx_q;

// A volatile variable used for interrupt-based receiving
extern volatile uint8_t rx_IRQ_data;

// This semaphore is signaled by UART2_IRQHandler when a byte arrives
extern osSemaphoreId_t sem_uartRx;

// Queue function declarations
void Q_Init(Q_T *q);
int Q_Empty(Q_T *q);
int Q_Full(Q_T *q);
int Q_Enqueue(Q_T *q, unsigned char d);
unsigned char Q_Dequeue(Q_T *q);

// UART setup and communication function declarations
void initUART2(uint32_t baud_rate);
void UART2_Transmit_Poll(uint8_t data);
uint8_t UART2_Receive_Poll(void);

// Interrupt Service Routine for UART2
void UART2_IRQHandler(void);

#endif // UART_H
