#include "MKL25Z4.h" // Device-specific register definitions
#include "uart.h"    // Header file with declarations
#include "cmsis_os2.h"
#include "motor.h"

// Define queue objects for transmit and receive
Q_T tx_q, rx_q;

// Volatile variable to store incoming data from interrupt
volatile uint8_t rx_IRQ_data = 0;

// Define the semaphore object
osSemaphoreId_t sem_uartRx = NULL;

// Initialize a queue to empty state
void Q_Init(Q_T *q) {
    for (unsigned int i = 0; i < Q_SIZE; i++) {
        q->DATA[i] = 0;
    }
    q->HEAD = q->TAIL = q->SIZE = 0;
}

// Return 1 if queue is empty, 0 otherwise
int Q_Empty(Q_T *q) {
    return q->SIZE == 0;
}

// Return 1 if queue is full, 0 otherwise
int Q_Full(Q_T *q) {
    return q->SIZE == Q_SIZE;
}

// Add a byte to the queue. Return 1 on success, 0 on failure
int Q_Enqueue(Q_T *q, unsigned char d) {
    if (Q_Full(q)) return 0;
    q->DATA[q->TAIL++] = d;
    q->TAIL %= Q_SIZE; // Wrap around
    q->SIZE++;
    return 1;
}

// Remove and return a byte from the queue. Return 0 if empty
unsigned char Q_Dequeue(Q_T *q) {
    if (Q_Empty(q)) return 0;
    unsigned char t = q->DATA[q->HEAD];
    q->DATA[q->HEAD++] = 0;
    q->HEAD %= Q_SIZE; // Wrap around
    q->SIZE--;
    return t;
}

// Interrupt handler for UART2 - handles both RX and TX
void UART2_IRQHandler(void) {
    NVIC_ClearPendingIRQ(UART2_IRQn); // Clear pending interrupt

    // TX interrupt - ready to transmit
    if (UART2->S1 & UART_S1_TDRE_MASK) {
        if (!Q_Empty(&tx_q)) {
            UART2->D = Q_Dequeue(&tx_q); // Send data from queue
        } else {
            UART2->C2 &= ~UART_C2_TIE_MASK; // Disable TX interrupt if queue is empty
        }
    }

    // RX interrupt - data received
    if (UART2->S1 & UART_S1_RDRF_MASK) {
        if (!Q_Full(&rx_q)) {
            Q_Enqueue(&rx_q, UART2->D); // Read received byte into queue
		
            osSemaphoreRelease(sem_uartRx); // Signal semaphore for new data
        } else {
            while (1); // RX queue overflow - halt (can be replaced with error handling)
        }
    }

    // Handle UART errors: overrun, noise, framing, or parity errors
    if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | 
                     UART_S1_FE_MASK | UART_S1_PF_MASK)) {
        // Just clear the flags by reading S1 and D (data)
        (void)UART2->D;
    }
}

// Initialize UART2 with specified baud rate
void initUART2(uint32_t baud_rate) {
    uint32_t divisor, bus_clock;

    // Enable clock to UART2 and PORTE (for pins)
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Configure RX pin (PTE23) for UART2 RX function (ALT4)
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

    // Disable TX and RX before configuring UART2
    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK)); 

    // Set baud rate (assuming 48 MHz system clock, 24 MHz bus clock)
    bus_clock = 48000000 / 2;
    divisor = bus_clock / (baud_rate * 16); // UART oversampled by 16

    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    UART2->BDL = UART_BDL_SBR(divisor);

    // No parity, 8-bit data, 1 stop bit
    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;

    // Set up NVIC interrupt controller
    NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // Enable RX interrupt only (TX enabled dynamically if needed)
    UART2->C2 |= UART_C2_RIE_MASK;

    // Initialize circular queues
    Q_Init(&rx_q);
    Q_Init(&tx_q);

    // Enable UART2 transmitter and receiver
    UART2->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
		
		// Create semaphore if not yet created
		if (!sem_uartRx) {
        sem_uartRx = osSemaphoreNew(Q_SIZE, 0, NULL);
    }
}

// Transmit a single byte using polling (waits until ready)
void UART2_Transmit_Poll(uint8_t data) {
    while (!(UART2->S1 & UART_S1_TDRE_MASK)); // Wait until transmit buffer is empty
    UART2->D = data; // Write byte to data register
}

// Receive a single byte using polling (waits until data received)
uint8_t UART2_Receive_Poll(void) {
    while (!(UART2->S1 & UART_S1_RDRF_MASK)); // Wait until data is received
    return UART2->D; // Return received byte
}
