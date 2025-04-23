// uart.c
// Implements a simple circular byte-queue for UART RX + TX (TX unused here),
// plus interrupt handler and basic polling routines.

#include "MKL25Z4.h"
#include "uart.h"
#include "cmsis_os2.h"

Q_T             tx_q, rx_q;          // Transmit / receive queues
osSemaphoreId_t sem_uartRx = NULL;   // Signals new RX data

// Initialize head/tail/size
void Q_Init(Q_T *q) {
    q->HEAD = q->TAIL = q->SIZE = 0;
}

int Q_Empty(Q_T *q) { return q->SIZE == 0; }
int Q_Full(Q_T *q)  { return q->SIZE == Q_SIZE; }

// Enqueue byte; returns 1=ok, 0=full
int Q_Enqueue(Q_T *q, uint8_t d) {
    if (Q_Full(q)) return 0;
    q->DATA[q->TAIL++] = d;
    q->TAIL %= Q_SIZE;
    q->SIZE++;
    return 1;
}

// Dequeue byte; returns 0 if empty
uint8_t Q_Dequeue(Q_T *q) {
    if (Q_Empty(q)) return 0;
    uint8_t d = q->DATA[q->HEAD++];
    q->HEAD %= Q_SIZE;
    q->SIZE--;
    return d;
}

// Set up UART2 at given baud, enable RX interrupt
void initUART2(uint32_t baud) {
    // Enable clocks
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    // Route PTE23 ? UART2_RX
    PORTE->PCR[UART_RX_PORTE23] = PORT_PCR_MUX(4);

    // Disable UART while configuring
    UART2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

    // Baud divisor (assuming bus = core/2)
    uint32_t bus = (SystemCoreClock / 2);
    uint32_t div = bus / (baud * 16);
    UART2->BDH = UART_BDH_SBR(div >> 8);
    UART2->BDL = UART_BDL_SBR(div);

    // 8N1, no parity
    UART2->C1 = UART2->S2 = UART2->C3 = 0;

    // Enable IRQ in NVIC
    NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);

    // Initialize RX queue
    Q_Init(&rx_q);

    // Enable RX interrupt + TX/RX
    UART2->C2 |= UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;

    // Create semaphore to signal RX bytes
    if (!sem_uartRx) {
        sem_uartRx = osSemaphoreNew(Q_SIZE, 0, NULL);
    }
}

// Poll-based transmit (blocks until ready)
void UART2_Transmit_Poll(uint8_t data) {
    while (!(UART2->S1 & UART_S1_TDRE_MASK));
    UART2->D = data;
}

// Poll-based receive (blocks until data available)
uint8_t UART2_Receive_Poll(void) {
    while (!(UART2->S1 & UART_S1_RDRF_MASK));
    return UART2->D;
}
